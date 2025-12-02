import os
import yaml
import json
import math

# ---- Add YAML degrees tag support ----
class Degrees(str):
    """Wrapper type for the custom YAML tag !degrees."""
    pass


def degrees_representer(dumper, data):
    return dumper.represent_scalar('!degrees', str(data))


yaml.add_representer(Degrees, degrees_representer)
# --------------------------------------


def generate_joint_limits(env_json_path, robot_type, output_dir):
    """
    Load environment.json and generate joint limit definitions.

    Parameters:
        env_json_path (str): Path to the environment.json file.
        robot_type (str): Robot type (e.g., ur5, ur10e).
        output_dir (str): Directory to write the output YAML.

    Returns:
        dict: The generated joint limit dictionary.
    """
    with open(env_json_path, "r", encoding="utf-8") as f:
        env_data = json.load(f)
    return load_joint_limits(robot_type, env_data, output_dir)


def rad_to_deg(rad):
    """Convert radians to degrees."""
    return rad * 180.0 / math.pi


def clean_joint_name(jname):
    """
    Normalize joint names exported in the AAS environment model.

    Removes prefixes/suffixes:
        - "Joint_"
        - "_Control"
        - "_Safety"
    """
    if jname.startswith("Joint_"):
        jname = jname[len("Joint_"):]
    if jname.endswith("_Control"):
        jname = jname[:-len("_Control")]
    if jname.endswith("_Safety"):
        jname = jname[:-len("_Safety")]
    return jname


def load_joint_limits(robot_type, env_data, output_dir):
    """
    Extract joint limit information from the AAS environment submodels
    and generate a ROS-compatible YAML file.

    Parameters:
        robot_type (str): Robot type (e.g. ur5).
        env_data (dict): Loaded JSON structure.
        output_dir (str): Output directory.

    Returns:
        dict: YAML structure containing all joint limits.
    """
    out_file = os.path.join(output_dir, f"{robot_type}_joint_limits.yaml")
    joints_tmp = {}

    # Parse joint limits from the Control Submodel
    for submodel in env_data.get("submodels", []):
        sid = submodel.get("idShort", "")
        if "Control" not in sid:
            continue

        for elem in submodel.get("submodelElements", []):
            raw_name = elem.get("idShort", "")
            jname = clean_joint_name(raw_name)

            props = {
                e["idShort"]: e.get("value")
                for e in elem.get("value", [])
                if isinstance(e, dict)
            }

            lower = props.get("lower")
            upper = props.get("upper")
            effort = props.get("effort")
            velocity = props.get("velocity")

            if jname not in joints_tmp:
                joints_tmp[jname] = {}

            if lower is not None:
                joints_tmp[jname]["min_position"] = rad_to_deg(float(lower))
            if upper is not None:
                joints_tmp[jname]["max_position"] = rad_to_deg(float(upper))
            if effort is not None:
                joints_tmp[jname]["max_effort"] = float(effort)
            if velocity is not None:
                joints_tmp[jname]["max_velocity"] = rad_to_deg(float(velocity))

    # Fill missing fields with defaults
    for j, vals in joints_tmp.items():
        vals.setdefault("has_acceleration_limits", False)
        vals.setdefault("has_effort_limits", "max_effort" in vals)
        vals.setdefault("has_position_limits", "min_position" in vals and "max_position" in vals)
        vals.setdefault("has_velocity_limits", "max_velocity" in vals)

    # Prepare output YAML structure
    final_yaml = {"joint_limits": {}}
    for j, vals in joints_tmp.items():
        formatted = {}
        for key, value in vals.items():
            if key in ["min_position", "max_position", "max_velocity"]:
                formatted[key] = Degrees(value)  # use custom YAML tag
            else:
                formatted[key] = value

        final_yaml["joint_limits"][j] = formatted

    os.makedirs(output_dir, exist_ok=True)
    with open(out_file, "w", encoding="utf-8") as f:
        yaml.dump(final_yaml, f, sort_keys=False, allow_unicode=True)

    return final_yaml
