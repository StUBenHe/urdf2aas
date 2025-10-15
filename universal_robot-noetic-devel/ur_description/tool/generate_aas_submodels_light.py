import os
import sys
import math
import yaml
import xml.etree.ElementTree as ET
import json
# os：处理文件路径和目录操作。
# json：用于生成 JSON 格式的输出。
# yaml：读取 YAML 配置文件（如关节限制、动力学参数等）。
# xml.etree.ElementTree：解析 URDF（XML 格式）文件。
# sys：用于处理命令行参数。

# 从指定路径加载 YAML 文件，如果文件不存在则返回空字典。


# 定义 !degrees 标签的解释器：将角度转换为弧度，
def degrees_constructor(loader, node):
    value = loader.construct_scalar(node)
    return float(value) * math.pi / 180.0

yaml.SafeLoader.add_constructor('!degrees', degrees_constructor)


# 加载yaml文件
def load_yaml(path):
    return yaml.safe_load(open(path, "r", encoding="utf-8")) if os.path.exists(path) else {}

# 解析 URDF 中的所有 link 和 joint，生成结构子模型。
# 每个 link 和 joint 作为一个子模型元素，用于描述机器人结构。
def generate_structure_submodel(urdf_root):
    elements = []
    for link in urdf_root.findall("link"):
        elements.append({
            "idShort": f"Link_{link.get('name')}",
            "modelType": "SubmodelElementCollection",
            "value": {"type": "link"}
        })
    for joint in urdf_root.findall("joint"):
        elements.append({
            "idShort": f"Joint_{joint.get('name')}",
            "modelType": "SubmodelElementCollection",
            "value": {
                "type": joint.get("type"),
                "parent": joint.find("parent").get("link"),
                "child": joint.find("child").get("link")
            }
        })
    return {"idShort": "StructureSubmodel", "submodelElements": elements}

# 生成控制子模型，包括每个关节的控制参数（如力矩effort、速度velocity、上下限）。
# 参数优先从 URDF 的 <limit> 标签中读取，缺失时从 joint_limits.yaml 补全。
def generate_control_submodel(urdf_root, joint_limits_yaml):
    elements = []
    for joint in urdf_root.findall("joint"):
        name = joint.get("name")
        limit_elem = joint.find("limit")
        yaml_limit = joint_limits_yaml.get(name, {})
        elements.append({
            "idShort": f"Joint_{name}_Control",
            "modelType": "SubmodelElementCollection",
            "value": {
                "lower": float(limit_elem.get("lower")) if limit_elem is not None and limit_elem.get("lower") else yaml_limit.get("lower"),
                "upper": float(limit_elem.get("upper")) if limit_elem is not None and limit_elem.get("upper") else yaml_limit.get("upper"),
                "effort": float(limit_elem.get("effort")) if limit_elem is not None and limit_elem.get("effort") else yaml_limit.get("effort"),
                "velocity": float(limit_elem.get("velocity")) if limit_elem is not None and limit_elem.get("velocity") else yaml_limit.get("velocity")
            }
        })
    return {"idShort": "ControlSubmodel", "submodelElements": elements}

# 生成运动学子模型，每个关节一个元素,DH-parament，参数来自 default_kinematics.yaml。
def generate_kinematics_submodel(kinematics_yaml):
    elements = []
    for joint, params in kinematics_yaml.items():
        elements.append({
            "idShort": f"Kinematics_{joint}",
            "modelType": "SubmodelElementCollection",
            "value": params
        })
    return {"idShort": "KinematicsSubmodel", "submodelElements": elements}

# 生成动力学子模型，提取 link 的质量mass、惯性矩阵inertia和原点坐标origin。
# 优先从 URDF 中读取，若缺失则从 physical_parameters.yaml 获取补充。
def generate_dynamics_submodel(urdf_root, physical_yaml):
    elements = []
    for link in urdf_root.findall("link"):
        name = link.get("name")
        inertial_elem = link.find("inertial")
        yaml_param = physical_yaml.get(name, {})
        value = {
            "mass": float(inertial_elem.find("mass").get("value")) if inertial_elem is not None and inertial_elem.find("mass") is not None else yaml_param.get("mass"),
            "inertia": inertial_elem.find("inertia").attrib if inertial_elem is not None and inertial_elem.find("inertia") is not None else yaml_param.get("inertia"),
            "origin": inertial_elem.find("origin").attrib if inertial_elem is not None and inertial_elem.find("origin") is not None else yaml_param.get("origin")
        }
        elements.append({
            "idShort": f"Dynamics_{name}",
            "modelType": "SubmodelElementCollection",
            "value": value
        })
    return {"idShort": "DynamicsSubmodel", "submodelElements": elements}

# 生成安全子模型，只提取关节的上下限（如安全限制范围）。
def generate_safety_submodel(urdf_root):
    elements = []
    for joint in urdf_root.findall("joint"):
        name = joint.get("name")
        limit = joint.find("limit")
        if limit is not None:
            elements.append({
                "idShort": f"Joint_{name}_Safety",
                "modelType": "SubmodelElementCollection",
                "value": {
                    "lower": float(limit.get("lower")) if limit.get("lower") else None,
                    "upper": float(limit.get("upper")) if limit.get("upper") else None
                }
            })
    return {"idShort": "SafetySubmodel", "submodelElements": elements}

# 生成可视化子模型，从 URDF 中提取 mesh 文件路径，并结合 visual_parameters.yaml 的颜色与材质信息。
def generate_visualization_submodel(urdf_root, visual_yaml):
    elements = []
    for link in urdf_root.findall("link"):
        name = link.get("name")
        visual = link.find("visual")
        mesh_elem = visual.find("geometry").find("mesh") if visual is not None and visual.find("geometry") is not None else None
        mesh_filename = mesh_elem.get("filename") if mesh_elem is not None else None
        extra = visual_yaml.get(name, {})
        elements.append({
            "idShort": f"Visual_{name}",
            "modelType": "SubmodelElementCollection",
            "value": {
                "mesh": mesh_filename,
                "color": extra.get("color"),
                "material": extra.get("material")
            }
        })
    return {"idShort": "VisualizationSubmodel", "submodelElements": elements}

# 保存子模型为 JSON 文件，写入到 output_dir 目录。
def save_submodel(submodel, name, output_dir):
    os.makedirs(output_dir, exist_ok=True)
    with open(os.path.join(output_dir, name), "w", encoding="utf-8") as f:
        json.dump(submodel, f, indent=2)

# 主函数：读取 URDF 和配置 YAML，生成各类 AAS 子模型并保存为 JSON。
def main(urdf_path, config_dir, output_dir):
    urdf_root = ET.parse(urdf_path).getroot()
    base = os.path.splitext(os.path.basename(urdf_path))[0]

    joint_limits = load_yaml(os.path.join(config_dir, "joint_limits.yaml"))
    kinematics = load_yaml(os.path.join(config_dir, "default_kinematics.yaml"))
    physical = load_yaml(os.path.join(config_dir, "physical_parameters.yaml"))
    visuals = load_yaml(os.path.join(config_dir, "visual_parameters.yaml"))

    save_submodel(generate_structure_submodel(urdf_root), f"{base}_structure_submodel.json", output_dir)
    save_submodel(generate_control_submodel(urdf_root, joint_limits), f"{base}_control_submodel.json", output_dir)
    save_submodel(generate_kinematics_submodel(kinematics), f"{base}_kinematics_submodel.json", output_dir)
    save_submodel(generate_dynamics_submodel(urdf_root, physical), f"{base}_dynamics_submodel.json", output_dir)
    save_submodel(generate_safety_submodel(urdf_root), f"{base}_safety_submodel.json", output_dir)
    save_submodel(generate_visualization_submodel(urdf_root, visuals), f"{base}_visualization_submodel.json", output_dir)

# 命令行入口，接收参数：URDF 路径、配置目录、输出目录
if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("用法: python generate_aas_submodels_light.py <urdf_file> <config_dir> <output_dir>")
        sys.exit(1)
    urdf_path = sys.argv[1]
    config_dir = sys.argv[2]
    output_dir = sys.argv[3]
    main(urdf_path, config_dir, output_dir)
