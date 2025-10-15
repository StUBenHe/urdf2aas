import os
import json
import yaml
from urdfpy import URDF

def load_yaml(file_path):
    if not os.path.exists(file_path):
        return {}
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)

def generate_structure_submodel(robot):
    elements = []
    for link in robot.links:
        elements.append({
            "idShort": f"Link_{link.name}",
            "modelType": "SubmodelElementCollection",
            "value": {
                "type": "link"
            }
        })
    for joint in robot.joints:
        elements.append({
            "idShort": f"Joint_{joint.name}",
            "modelType": "SubmodelElementCollection",
            "value": {
                "type": joint.joint_type,
                "parent": joint.parent,
                "child": joint.child
            }
        })
    return {"idShort": "StructureSubmodel", "submodelElements": elements}

def generate_control_submodel(robot, joint_limits):
    elements = []
    for joint in robot.joints:
        limit = joint.limit
        limits_yaml = joint_limits.get(joint.name, {})
        elements.append({
            "idShort": f"Joint_{joint.name}_Control",
            "modelType": "SubmodelElementCollection",
            "value": {
                "lower": limit.lower if limit else limits_yaml.get('lower', None),
                "upper": limit.upper if limit else limits_yaml.get('upper', None),
                "effort": limit.effort if limit else limits_yaml.get('effort', None),
                "velocity": limit.velocity if limit else limits_yaml.get('velocity', None)
            }
        })
    return {"idShort": "ControlSubmodel", "submodelElements": elements}

def generate_kinematics_submodel(kinematics_yaml):
    elements = []
    for joint_name, params in kinematics_yaml.items():
        elements.append({
            "idShort": f"Kinematics_{joint_name}",
            "modelType": "SubmodelElementCollection",
            "value": params
        })
    return {"idShort": "KinematicsSubmodel", "submodelElements": elements}

def generate_dynamics_submodel(robot, physical_params):
    elements = []
    for link in robot.links:
        inertial = link.inertial
        params_yaml = physical_params.get(link.name, {})
        dynamics = {
            "mass": inertial.mass if inertial else params_yaml.get('mass'),
            "inertia": inertial.inertia.tolist() if inertial else params_yaml.get('inertia'),
            "origin": inertial.origin.tolist() if inertial else params_yaml.get('origin')
        }
        elements.append({
            "idShort": f"Dynamics_{link.name}",
            "modelType": "SubmodelElementCollection",
            "value": dynamics
        })
    return {"idShort": "DynamicsSubmodel", "submodelElements": elements}

def generate_safety_submodel(robot):
    elements = []
    for joint in robot.joints:
        limit = joint.limit
        if limit:
            elements.append({
                "idShort": f"Joint_{joint.name}_Safety",
                "modelType": "SubmodelElementCollection",
                "value": {
                    "lower": limit.lower,
                    "upper": limit.upper
                }
            })
    return {"idShort": "SafetySubmodel", "submodelElements": elements}

def generate_visualization_submodel(robot, visual_params):
    elements = []
    for link in robot.links:
        visuals = link.visuals
        vis_param = visual_params.get(link.name, {})
        for i, vis in enumerate(visuals):
            elements.append({
                "idShort": f"Visual_{link.name}_{i}",
                "modelType": "SubmodelElementCollection",
                "value": {
                    "mesh": vis.geometry.mesh.filename if hasattr(vis.geometry, 'mesh') and vis.geometry.mesh else None,
                    "color": vis_param.get('color'),
                    "material": vis_param.get('material')
                }
            })
    return {"idShort": "VisualizationSubmodel", "submodelElements": elements}

def save_submodel(submodel, name, output_dir):
    os.makedirs(output_dir, exist_ok=True)
    with open(os.path.join(output_dir, name + ".json"), "w") as f:
        json.dump(submodel, f, indent=2)

def main(urdf_path, yaml_dir, output_dir="output"):
    robot = URDF.load(urdf_path)
    joint_limits = load_yaml(os.path.join(yaml_dir, "joint_limits.yaml"))
    kinematics = load_yaml(os.path.join(yaml_dir, "default_kinematics.yaml"))
    physical = load_yaml(os.path.join(yaml_dir, "physical_parameters.yaml"))
    visual = load_yaml(os.path.join(yaml_dir, "visual_parameters.yaml"))

    base_name = os.path.splitext(os.path.basename(urdf_path))[0]

    save_submodel(generate_structure_submodel(robot), f"{base_name}_structure_submodel", output_dir)
    save_submodel(generate_control_submodel(robot, joint_limits), f"{base_name}_control_submodel", output_dir)
    save_submodel(generate_kinematics_submodel(kinematics), f"{base_name}_kinematics_submodel", output_dir)
    save_submodel(generate_dynamics_submodel(robot, physical), f"{base_name}_dynamics_submodel", output_dir)
    save_submodel(generate_safety_submodel(robot), f"{base_name}_safety_submodel", output_dir)
    save_submodel(generate_visualization_submodel(robot, visual), f"{base_name}_visualization_submodel", output_dir)

if __name__ == "__main__":
    # 示例：用于 ur3
    main("urdf/ur3.urdf", "config/ur3")
