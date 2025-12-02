import os
import json
from lxml import etree
from urdfpy import URDF
import numpy as np
from generate_aas_submodels_ur import (
    load_yaml,
    save_submodel,
    generate_structure_submodel,
    generate_control_submodel,
    generate_kinematics_submodel,
    generate_dynamics_submodel,
    generate_safety_submodel,
    generate_visualization_submodel,
    assemble_environment_v3,
)

# === Joint Limit Compatibility Patch ===
def patch_limit_compatibility(robot):
    """
    Fix compatibility issues in URDF joint limit definitions across urdfpy versions.

    - urdfpy may represent joint.limit as None / dict / object
    - Values may be strings, lists, ndarrays, or empty
    - Skip fixed/floating/planar joints
    - Provide default [-2π, 2π] for continuous joints
    - Ensure all joints end up with a writable object containing:
        .lower, .upper, .effort, .velocity
    """
    import numpy as np
    TWO_PI = 2.0 * np.pi

    def _to_float(val, default=None):
        try:
            if val is None:
                return default
            if isinstance(val, (list, tuple, np.ndarray)):
                if len(val) == 0:
                    return default
                val = val[0]
            if isinstance(val, str):
                val = val.strip()
                if val == "":
                    return default
            return float(val)
        except Exception:
            return default

    for joint in getattr(robot, "joints", []):
        jt = getattr(joint, "joint_type", "fixed")
        if jt in ("fixed", "floating", "planar"):
            continue

        lim = getattr(joint, "limit", None)

        if isinstance(lim, dict):
            lower    = _to_float(lim.get("lower"),    None)
            upper    = _to_float(lim.get("upper"),    None)
            effort   = _to_float(lim.get("effort"),   None)
            velocity = _to_float(lim.get("velocity"), None)
        else:
            lower    = _to_float(getattr(lim, "lower",    None), None)
            upper    = _to_float(getattr(lim, "upper",    None), None)
            effort   = _to_float(getattr(lim, "effort",   None), None)
            velocity = _to_float(getattr(lim, "velocity", None), None)

        if jt == "continuous":
            if lower is None: lower = -TWO_PI
            if upper is None: upper =  TWO_PI
        else:
            if lower is None: lower = -np.pi
            if upper is None: upper =  np.pi

        if effort is None:
            effort = 0.0
        if velocity is None:
            velocity = 1.0

        if lim is None or not hasattr(lim, "__dict__"):
            class _Limit: pass
            lim_obj = _Limit()
            setattr(joint, "limit", lim_obj)
        else:
            lim_obj = lim

        setattr(lim_obj, "lower",    float(lower))
        setattr(lim_obj, "upper",    float(upper))
        setattr(lim_obj, "effort",   float(effort))
        setattr(lim_obj, "velocity", float(velocity))


# === Kinematics Submodel Fallback Generator ===
def generate_kinematics_submodel_fallback(kinematics_yaml, robot=None):
    """
    Generate a minimal Kinematics Submodel if the YAML file is missing.

    Uses joint axes and origins extracted from the URDF model.
    """
    import numpy as np

    def _safe_list(v):
        if v is None:
            return [0, 0, 0]
        if isinstance(v, np.ndarray):
            return v.tolist()
        if isinstance(v, (list, tuple)):
            return list(v)
        return [float(v), 0, 0] if isinstance(v, (int, float)) else [0, 0, 0]

    elements = []

    if not kinematics_yaml and robot is not None:
        print("Warning: Kinematics YAML missing, generating minimal fallback parameters.")
        for joint in robot.joints:
            if joint.joint_type in ["revolute", "continuous", "prismatic"]:

                xyz, rpy = [0, 0, 0], [0, 0, 0]
                if hasattr(joint, "origin") and joint.origin is not None:
                    try:
                        if hasattr(joint.origin, "xyz"):
                            xyz = _safe_list(joint.origin.xyz)
                        elif hasattr(joint.origin, "translation"):
                            xyz = _safe_list(joint.origin.translation)
                        if hasattr(joint.origin, "rpy"):
                            rpy = _safe_list(joint.origin.rpy)
                    except:
                        xyz, rpy = [0, 0, 0], [0, 0, 0]

                axis_val = getattr(joint, "axis", [0, 0, 1])
                axis_val = _safe_list(axis_val)

                elements.append({
                    "idShort": f"Kinematics_{joint.name}",
                    "modelType": "SubmodelElementCollection",
                    "value": {
                        "axis": axis_val,
                        "type": joint.joint_type,
                        "origin": {"xyz": xyz, "rpy": rpy}
                    }
                })
    else:
        for joint_name, params in kinematics_yaml.items():
            elements.append({
                "idShort": f"Kinematics_{joint_name}",
                "modelType": "SubmodelElementCollection",
                "value": params
            })

    return {"idShort": "KinematicsSubmodel", "submodelElements": elements}


def restore_package_uri_recursive(obj, package_root, pkg_prefix="package://igus_rebel_description_ros2/"):
    """
    Recursively restore absolute file paths to package:// URIs inside a JSON structure.
    """
    if isinstance(obj, dict):
        for k, v in obj.items():
            obj[k] = restore_package_uri_recursive(v, package_root, pkg_prefix)
        return obj

    elif isinstance(obj, list):
        return [restore_package_uri_recursive(x, package_root, pkg_prefix) for x in obj]

    elif isinstance(obj, str):
        normalized = obj.replace("\\", "/")
        package_root = package_root.replace("\\", "/")

        if normalized.startswith(package_root):
            rel = normalized[len(package_root):]
            if rel.startswith("/"):
                rel = rel[1:]
            return pkg_prefix + rel
        return obj

    else:
        return obj


# === IGUS Rebel AAS Submodel Generation Pipeline ===
def main_igus_rebel():
    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    igus_root = os.path.join(base_dir, "types", "igus_rebel_description_ros2")
    submodel_root = os.path.join(base_dir, "types", "submodel", "igus_rebel_6dof")

    urdf_path = os.path.join(igus_root, "urdf", "igus_rebel.urdf")
    yaml_dir = os.path.join(igus_root, "config", "igus_rebel_6dof")
    output_dir = submodel_root

    if not os.path.exists(urdf_path):
        print(f"Error: URDF file not found: {urdf_path}")
        return

    os.makedirs(yaml_dir, exist_ok=True)
    os.makedirs(output_dir, exist_ok=True)

    print(f"Loading IGUS Rebel URDF: {urdf_path}")

    with open(urdf_path, "r", encoding="utf-8") as f:
        urdf_text = f.read().replace("\\", "/")

    package_root = os.path.join(base_dir, "types", "igus_rebel_description_ros2").replace("\\", "/")
    urdf_text = urdf_text.replace("package://igus_rebel_description_ros2/", package_root + "/")

    parser = etree.XMLParser(remove_blank_text=True)
    root = etree.fromstring(urdf_text.encode("utf-8"), parser=parser)

    import trimesh
    _original_trimesh_load = trimesh.load
    trimesh.load = lambda *a, **kw: trimesh.Trimesh()

    try:
        robot = URDF._from_xml(root, None)
    finally:
        trimesh.load = _original_trimesh_load

    patch_limit_compatibility(robot)
    print(f"Model loaded successfully: {robot.name}, {len(robot.links)} links, {len(robot.joints)} joints.")

    joint_limits = load_yaml(os.path.join(yaml_dir, "joint_limits.yaml"))
    kinematics = load_yaml(os.path.join(yaml_dir, "default_kinematics.yaml"))
    physical = load_yaml(os.path.join(yaml_dir, "physical_parameters.yaml"))
    visual = load_yaml(os.path.join(yaml_dir, "visual_parameters.yaml"))
    base_name = "igus_rebel_6dof"
    control_template_path = os.path.join(yaml_dir, f"{base_name}_control.json")

    structure = generate_structure_submodel(base_name, root)
    structure = restore_package_uri_recursive(structure, package_root)
    save_submodel(structure, f"{base_name}_structure_submodel", output_dir)

    save_submodel(generate_control_submodel(robot, control_template_path, joint_limits),
                  f"{base_name}_control_submodel", output_dir)

    save_submodel(generate_kinematics_submodel_fallback(kinematics, robot),
                  f"{base_name}_kinematics_submodel", output_dir)

    save_submodel(generate_dynamics_submodel(robot, physical),
                  f"{base_name}_dynamics_submodel", output_dir)

    save_submodel(generate_safety_submodel(robot),
                  f"{base_name}_safety_submodel", output_dir)

    save_submodel(generate_visualization_submodel(robot, visual),
                  f"{base_name}_visualization_submodel", output_dir)

    assemble_environment_v3(base_name, output_dir)
    print(f"All AAS Submodels for IGUS Rebel generated successfully at: {output_dir}")


if __name__ == "__main__":
    main_igus_rebel()
