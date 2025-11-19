import os
import json
import yaml
import re
from lxml import etree
from urdfpy import URDF
import numpy as np
import uuid
import os
import json
from pathlib import Path

#Guess the appropriate AAS valueType (e.g., xs:boolean, xs:double, xs:string) based on Python data type.
def _guess_value_type(v):
    if isinstance(v, bool):
        return "xs:boolean"
    if isinstance(v, int) or isinstance(v, float):
        return "xs:double"
    # 其它一律存成字符串
    return "xs:string"

#Wrap a single scalar, list, or dict into an AAS Property element.
def _to_property(id_short, v):
    # 列表/字典 -> 序列化存成字符串，避免再次触发“期望数组”的错误
    if isinstance(v, (list, tuple, dict)):
        value = json.dumps(v, ensure_ascii=False)
        value_type = "xs:string"
    else:
        value = str(v)
        value_type = _guess_value_type(v)
    return {
        "modelType": "Property",
        "idShort": id_short,
        "valueType": value_type,
        "value": value
    }

#Convert a mesh file path into an AAS File element with MIME type.
def _to_file(id_short, path_str):
    # 依据扩展名给出一个合理的 contentType
    ext = Path(path_str).suffix.lower()
    mime = {
        ".stl": "model/stl",
        ".dae": "model/vnd.collada+xml",
        ".obj": "model/obj",
        ".glb": "model/gltf-binary",
        ".gltf": "model/gltf+json"
    }.get(ext, "application/octet-stream")
    return {
        "modelType": "File",
        "idShort": id_short,
        "contentType": mime,
        "value": path_str
    }

#Convert a key-value dict into a SubmodelElementCollection value list.
def _kv_to_smc_value(obj):
    """
    把 dict 转为 AAS 合法的 SMC.value（数组）
    规则：
      - "mesh" -> File
      - 其它键：
          - 标量 -> Property
          - 列表/字典 -> Property(字符串化) 以保证 AASX 不再要求子层也是数组
    """
    items = []
    for k, v in obj.items():
        if k == "mesh" and isinstance(v, str) and v:
            items.append(_to_file("mesh", v))
        else:
            items.append(_to_property(k, v))
    return items

#Fix AAS submodel structure recursively to enforce valid SMC array format.
def fix_submodel_structure(node):
    """递归修正所有 SubmodelElementCollection 的 value 为 list，避免空对象。"""
    if isinstance(node, dict):
        # 修正当前 SMC
        if node.get("modelType") == "SubmodelElementCollection":
            val = node.get("value")
            if isinstance(val, dict):
                node["value"] = [
                    {
                        "modelType": "Property",
                        "idShort": k,
                        "valueType": "xs:string" if not isinstance(v, (int, float)) else "xs:double",
                        "value": json.dumps(v, ensure_ascii=False)
                        if isinstance(v, (list, dict)) else str(v)
                    }
                    for k, v in val.items()
                    if v not in [None, {}, []]  # 跳过空值
                ]
        # 递归子项
        for v in node.values():
            fix_submodel_structure(v)
    elif isinstance(node, list):
        for v in node:
            fix_submodel_structure(v)

#Recursively replace absolute file paths containing base_marker with relative package:// UR-style paths.
def make_paths_relative(node, base_marker="ur_description"):
    """
    递归扫描所有 mesh 字段，把绝对路径改成以 package://ur_description/... 为前缀的相对路径
    """
    if isinstance(node, dict):
        for k, v in list(node.items()):
            if isinstance(v, str) and base_marker in v:
                # 截取 ur_description 及其后面的路径部分
                relative_part = v.split(base_marker, 1)[-1].lstrip("/\\")
                node[k] = f"package://{base_marker}/{relative_part}"
            else:
                make_paths_relative(v, base_marker)
    elif isinstance(node, list):
        for item in node:
            make_paths_relative(item, base_marker)

#Recursively ensure all SubmodelElementCollection values are lists instead of dicts.
def _fix_smc_value_inplace(node):
    """
    递归修正：凡是 modelType == 'SubmodelElementCollection' 的元素，
    若 value 不是 list，则改造成 list；子层继续递归。
    """
    if isinstance(node, dict):
        # 修正当前层
        if node.get("modelType") == "SubmodelElementCollection":
            val = node.get("value")
            if isinstance(val, dict):          # 核心修复：dict -> list
                node["value"] = _kv_to_smc_value(val)
            elif not isinstance(val, list):    # None 或标量 -> 包一层 Property
                node["value"] = [_to_property("value", val)]
        # 递归子字段
        for k, v in list(node.items()):
            _fix_smc_value_inplace(v)
    elif isinstance(node, list):
        for it in node:
            _fix_smc_value_inplace(it)

#Wrap raw content into a valid AAS Submodel structure with required metadata (id, idShort, semanticId).
def _wrap_as_submodel(content, base_name, fallback_idshort):
    """
    把 {idShort, submodelElements: ...} 包成 AASX 可识别的 Submodel 对象。
    并统一把所有 SMC 的 value 修正成数组。
    """
    # 1) 确保有 idShort 与 submodelElements
    id_short = content.get("idShort", fallback_idshort)
    sm_elements = content.get("submodelElements", [])

    # 2) 递归修正所有 SMC.value
    _fix_smc_value_inplace(sm_elements)

    # 3) 封装成 Submodel（AASX 接受 "modelType": "Submodel" 这一写法）
    return {
        "modelType": "Submodel",
        "id": f"urn:submodel:robot:{id_short}:{base_name}:safe",
        "idShort": id_short,
        "category": f"Robot:{base_name.upper()}",
        "submodelElements": sm_elements
    }

# === 自定义 YAML 解析器，支持 ROS 风格的 !degrees ===
#Custom YAML constructor to support ROS-style !degrees notation and convert to radians.
def degrees_constructor(loader, node):
    value = loader.construct_scalar(node)
    try:
        return float(value)
    except ValueError:
        return value
yaml.SafeLoader.add_constructor('!degrees', degrees_constructor)

# ====== 修复 numpy 兼容性问题 ======
if not hasattr(np, 'float'):
    np.float = float
# ==================================

# ===============================
# 📘 工具函数
# ===============================
#Load YAML configuration file safely and return a dictionary.
def load_yaml(file_path):
    if not os.path.exists(file_path):
        return {}
    with open(file_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)

#Save the given submodel as a formatted JSON file to the output directory.
def save_submodel(submodel, name, output_dir):
    os.makedirs(output_dir, exist_ok=True)
    file_path = os.path.join(output_dir, name + ".json")
    with open(file_path, "w", encoding="utf-8") as f:
        json.dump(submodel, f, indent=2, ensure_ascii=False)
    print(f"💾 已生成: {file_path}")


#Convert numpy arrays or iterable objects into plain Python lists for safe JSON serialization.
def safe_list(value):
    if value is None:
        return None
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (list, tuple)):
        return list(value)
    return value


# ===============================
# 🧩 子模型生成函数
# ===============================
#Build the Structure Submodel from URDF data, including links, joints, inertial, and mesh info.
def generate_structure_submodel(robot_name, xml_root):
    """
    生成符合 AAS 结构要求的 Structure Submodel。
    - visuals 和 collisions 分离成集合
    - origin 作为独立字段
    - 自动过滤 inertia-only link
    """

    def matrix_to_str(matrix):
        return json.dumps(matrix, ensure_ascii=False)

    structure_submodel = {
        "modelType": "Submodel",
        "id": f"urn:submodel:robot:{robot_name}:Structure",
        "idShort": f"{robot_name}_StructureSubmodel",
        "category": f"Robot:{robot_name}",
        "submodelElements": []
    }

    # 解析 links
    for link in xml_root.findall("link"):
        link_name = link.attrib["name"]

        link_collection = {
            "idShort": f"Link_{link_name}",
            "modelType": "SubmodelElementCollection",
            "value": []
        }

        # 添加基本属性
        link_collection["value"].append({
            "modelType": "Property",
            "idShort": "type",
            "valueType": "xs:string",
            "value": "link"
        })
        # inertia 节点
        inertia_node = link.find("inertial/inertia")
        mass_node = link.find("inertial/mass")
        inertia_origin_node = link.find("inertial/origin")

        if inertia_node is not None:
            Ixx = float(inertia_node.attrib.get("ixx", 0))
            Iyy = float(inertia_node.attrib.get("iyy", 0))
            Izz = float(inertia_node.attrib.get("izz", 0))
            Ixy = float(inertia_node.attrib.get("ixy", 0))
            Ixz = float(inertia_node.attrib.get("ixz", 0))
            Iyz = float(inertia_node.attrib.get("iyz", 0))

            inertia_matrix = [
                [Ixx, Ixy, Ixz],
                [Ixy, Iyy, Iyz],
                [Ixz, Iyz, Izz]
            ]

            link_collection["value"].append({
                "modelType": "Property",
                "idShort": "inertia",
                "valueType": "xs:string",
                "value": matrix_to_str(inertia_matrix)
            })

        # inertia origin
        if inertia_origin_node is not None:
            origin = {
                "xyz": inertia_origin_node.attrib.get("xyz", "0 0 0").split(),
                "rpy": inertia_origin_node.attrib.get("rpy", "0 0 0").split()
            }
            link_collection["value"].append({
                "modelType": "Property",
                "idShort": "inertia_origin",
                "valueType": "xs:string",
                "value": json.dumps(origin)
            })

        # mass
        if mass_node is not None:
            link_collection["value"].append({
                "modelType": "Property",
                "idShort": "mass",
                "valueType": "xs:double",
                "value": mass_node.attrib.get("value", "0")
            })

        # visuals
        visuals_node = link.findall("visual")
        if visuals_node:
            visuals_collection = {
                "idShort": "visuals",
                "modelType": "SubmodelElementCollection",
                "value": []
            }
            for idx, v in enumerate(visuals_node):
                mesh_node = v.find("geometry/mesh")
                origin_node = v.find("origin")
                mesh_path = mesh_node.attrib.get("filename", "") if mesh_node is not None else ""

                origin = {
                    "xyz": origin_node.attrib.get("xyz", "0 0 0").split(),
                    "rpy": origin_node.attrib.get("rpy", "0 0 0").split()
                } if origin_node is not None else None

                visuals_collection["value"].append({
                    "idShort": f"visual_{idx}",
                    "modelType": "SubmodelElementCollection",
                    "value": [
                        {"modelType": "Property", "idShort": "mesh", "valueType": "xs:string", "value": mesh_path},
                        {"modelType": "Property", "idShort": "origin", "valueType": "xs:string", "value": json.dumps(origin)}
                    ]
                })
            link_collection["value"].append(visuals_collection)

        # collisions
        collisions_node = link.findall("collision")
        if collisions_node:
            collisions_collection = {
                "idShort": "collisions",
                "modelType": "SubmodelElementCollection",
                "value": []
            }
            for idx, c in enumerate(collisions_node):
                mesh_node = c.find("geometry/mesh")
                origin_node = c.find("origin")
                mesh_path = mesh_node.attrib.get("filename", "") if mesh_node is not None else ""

                origin = {
                    "xyz": origin_node.attrib.get("xyz", "0 0 0").split(),
                    "rpy": origin_node.attrib.get("rpy", "0 0 0").split()
                } if origin_node is not None else None

                collisions_collection["value"].append({
                    "idShort": f"collision_{idx}",
                    "modelType": "SubmodelElementCollection",
                    "value": [
                        {"modelType": "Property", "idShort": "mesh", "valueType": "xs:string", "value": mesh_path},
                        {"modelType": "Property", "idShort": "origin", "valueType": "xs:string", "value": json.dumps(origin)}
                    ]
                })
            link_collection["value"].append(collisions_collection)

        structure_submodel["submodelElements"].append(link_collection)

    # joints
    for joint in xml_root.findall("joint"):
        joint_name = joint.attrib["name"]
        joint_type = joint.attrib.get("type", "fixed")
        parent = joint.find("parent").attrib.get("link", "")
        child = joint.find("child").attrib.get("link", "")

        joint_collection = {
            "idShort": f"Joint_{joint_name}",
            "modelType": "SubmodelElementCollection",
            "value": [
                {"modelType": "Property", "idShort": "type", "valueType": "xs:string", "value": joint_type},
                {"modelType": "Property", "idShort": "parent", "valueType": "xs:string", "value": parent},
                {"modelType": "Property", "idShort": "child", "valueType": "xs:string", "value": child},
            ]
        }

        origin_node = joint.find("origin")
        if origin_node is not None:
            origin = {
                "xyz": origin_node.attrib.get("xyz", "0 0 0").split(),
                "rpy": origin_node.attrib.get("rpy", "0 0 0").split()
            }
            joint_collection["value"].append({
                "modelType": "Property",
                "idShort": "origin",
                "valueType": "xs:string",
                "value": json.dumps(origin)
            })

        structure_submodel["submodelElements"].append(joint_collection)

    return structure_submodel

#Generate Control Submodel with controller configuration, joint limits, and transmission info
def generate_control_submodel(robot, control_template_path, joint_limits):
    """从 URDF + 控制模板（JSON）生成完整控制子模型"""
    controllers_data = {}
    if os.path.exists(control_template_path):
        with open(control_template_path, "r", encoding="utf-8") as f:
            control_template = json.load(f)
        for sm_element in control_template.get("submodelElements", []):
            if sm_element["idShort"] == "Controllers":
                for controller in sm_element["value"]:
                    ctrl_id = controller["idShort"]
                    ctrl_params = {}
                    for v in controller["value"]:
                        if v["idShort"] == "mode":
                            ctrl_params["mode"] = v["value"]
                        elif v["idShort"] == "updateRateHz":
                            ctrl_params["updateRateHz"] = float(v["value"])
                        elif v["idShort"] == "gains":
                            ctrl_params["gains"] = {g["idShort"]: float(g["value"]) for g in v["value"]}
                        elif v["idShort"] == "targets":
                            ctrl_params["targets"] = [t["value"] for t in v["value"]]
                    controllers_data[ctrl_id] = ctrl_params

    elements = []
    for joint in robot.joints:
        limit = joint.limit
        limits_yaml = joint_limits.get(joint.name, {})
        joint_entry = {
            "lower": limit.lower if limit else limits_yaml.get("lower"),
            "upper": limit.upper if limit else limits_yaml.get("upper"),
            "effort": limit.effort if limit else limits_yaml.get("effort"),
            "velocity": limit.velocity if limit else limits_yaml.get("velocity"),
            # ✅ transmission 信息
            "transmission": {
                "type": "transmission_interface/SimpleTransmission",
                "mechanicalReduction": 1.0,
                "hardwareInterface": "hardware_interface/PositionJointInterface"
            }
        }

        # 控制器匹配
        for ctrl_id, params in controllers_data.items():
            if joint.name in params.get("targets", []):
                joint_entry.update({
                    "controller_id": ctrl_id,
                    "mode": params.get("mode"),
                    "updateRateHz": params.get("updateRateHz"),
                    "gains": params.get("gains")
                })

        elements.append({
            "idShort": f"Joint_{joint.name}_Control",
            "modelType": "SubmodelElementCollection",
            "value": joint_entry
        })

    return {"idShort": "ControlSubmodelFull", "submodelElements": elements}


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
            "mass": inertial.mass if inertial else params_yaml.get("mass"),
            "inertia": safe_list(inertial.inertia if inertial else params_yaml.get("inertia")),
            "origin": safe_list(inertial.origin if inertial else params_yaml.get("origin"))
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
        if joint.limit:
            elements.append({
                "idShort": f"Joint_{joint.name}_Safety",
                "modelType": "SubmodelElementCollection",
                "value": {"lower": joint.limit.lower, "upper": joint.limit.upper}
            })
    return {"idShort": "SafetySubmodel", "submodelElements": elements}


def generate_visualization_submodel(robot, visual_params):
    """包含 mesh、材质、颜色的可视化子模型"""
    elements = []
    for link in robot.links:
        vis_param = visual_params.get(link.name, {})
        for i, vis in enumerate(link.visuals):
            mesh_file = None
            if hasattr(vis.geometry, "mesh") and vis.geometry.mesh:
                mesh_file = vis.geometry.mesh.filename

            # ✅ 读取材质信息
            material = getattr(vis, "material", None)
            mat_name = getattr(material, "name", None)
            color = None
            if material and hasattr(material, "color") and material.color is not None:
                if hasattr(material.color, "rgba"):
                    color = safe_list(material.color.rgba)
                else:
                    color = safe_list(material.color)

            elements.append({
                "idShort": f"Visual_{link.name}_{i}",
                "modelType": "SubmodelElementCollection",
                "value": {
                    "mesh": mesh_file,
                    "material": mat_name or vis_param.get("material"),
                    "color": color or vis_param.get("color")
                }
            })
    return {"idShort": "VisualizationSubmodel", "submodelElements": elements}


import uuid
import json
import os

def assemble_environment_v3(base_name, output_dir):
    """
    汇总所有子模型并修正结构，每个子模型带上标签前缀（如 UR3_Structure）。
    生成文件可直接通过 “Import Submodel from JSON” 导入。
    """
    submodel_files = [
        f"{base_name}_structure_submodel.json",
        f"{base_name}_control_submodel.json",
        f"{base_name}_kinematics_submodel.json",
        f"{base_name}_dynamics_submodel.json",
        f"{base_name}_safety_submodel.json",
        f"{base_name}_visualization_submodel.json",
    ]

    submodels = []
    base_upper = base_name.upper()

    for file_name in submodel_files:
        path = os.path.join(output_dir, file_name)
        if not os.path.exists(path):
            print(f"⚠️ 缺少子模型文件: {file_name}")
            continue

        with open(path, "r", encoding="utf-8") as f:
            content = json.load(f)

        # 🧩 自动修复内部结构
        fix_submodel_structure(content)

        # 提取子模型类型标签，例如 "structure"
        label_part = file_name.replace(f"{base_name}_", "").replace("_submodel.json", "")
        label_upper = label_part.capitalize()

        # 🔖 拼接成标签名
        id_short = f"{base_upper}_{label_upper}Submodel"
        sm_id = f"urn:submodel:robot:{base_upper}:{label_upper}"

        submodels.append({
            "modelType": "Submodel",
            "id": sm_id,
            "idShort": id_short,
            "category": f"Robot:{base_upper}",
            "submodelElements": content.get("submodelElements", [])
        })

    environment = {"submodels": submodels}
    # ✅ 在写入文件之前执行路径相对化修正
    make_paths_relative(environment)

    output_path = os.path.join(output_dir, f"{base_name}_environment.json")

    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(environment, f, indent=2, ensure_ascii=False)

    print(f"✅ 已生成带标签的可导入文件: {output_path}")

# ===============================
# ⚙️ 主逻辑
# ===============================
def main(urdf_path, yaml_dir, output_dir="output"):
    print(f"\n=== Generating AAS Submodels for {os.path.basename(urdf_path).split('.')[0]} ===")

    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    urcell_root = base_dir.replace("\\", "/")
    package_root = os.path.join(urcell_root, "types", "ur_description").replace("\\", "/")

    with open(urdf_path, "r", encoding="utf-8") as f:
        urdf_text = f.read().replace("\\", "/")
    urdf_text = urdf_text.replace("package://ur_description/", package_root + "/").replace("//", "/")

    if "package://" in urdf_text:
        print("⚠️ 部分 package:// 路径未替换（保留 ROS 兼容格式）。")
    else:
        print("✅ 路径解析成功，所有 mesh 均可解析。")

    debug_dir = os.path.join(os.path.dirname(__file__), "output")
    os.makedirs(debug_dir, exist_ok=True)
    debug_path = os.path.join(debug_dir, f"debug_{os.path.basename(urdf_path).split('.')[0]}_fixed.urdf")
    with open(debug_path, "w", encoding="utf-8") as f:
        f.write(urdf_text)
    print(f"📄 已输出调试文件: {debug_path}")

    parser = etree.XMLParser(remove_blank_text=True)
    root = etree.fromstring(urdf_text.encode("utf-8"), parser=parser)
    robot = URDF._from_xml(root, None)
    print(f"🤖 成功加载 URDF 模型: {robot.name}, 包含 {len(robot.links)} 个 link, {len(robot.joints)} 个 joint。")

    joint_limits = load_yaml(os.path.join(yaml_dir, "joint_limits.yaml"))
    kinematics = load_yaml(os.path.join(yaml_dir, "default_kinematics.yaml"))
    physical = load_yaml(os.path.join(yaml_dir, "physical_parameters.yaml"))
    visual = load_yaml(os.path.join(yaml_dir, "visual_parameters.yaml"))
    base_name = os.path.splitext(os.path.basename(urdf_path))[0]
    control_template_path = os.path.join(yaml_dir, f"{base_name}_control.json")

    save_submodel(generate_structure_submodel(robot, root),
                  f"{base_name}_structure_submodel", output_dir)

    save_submodel(generate_control_submodel(robot, control_template_path, joint_limits), f"{base_name}_control_submodel", output_dir)
    save_submodel(generate_kinematics_submodel(kinematics), f"{base_name}_kinematics_submodel", output_dir)
    save_submodel(generate_dynamics_submodel(robot, physical), f"{base_name}_dynamics_submodel", output_dir)
    save_submodel(generate_safety_submodel(robot), f"{base_name}_safety_submodel", output_dir)
    save_submodel(generate_visualization_submodel(robot, visual), f"{base_name}_visualization_submodel", output_dir)
    assemble_environment_v3(base_name, output_dir)

    print(f"🎯 已完成 {base_name} 的所有 AAS 子模型生成。\n")



if __name__ == "__main__":
    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    urdf_root = os.path.join(base_dir, "types", "ur_description", "urdf")
    config_root = os.path.join(base_dir, "types", "ur_description", "config")
    submodel_root = os.path.join(base_dir, "types", "submodel")

    ur_names = [
        "ur3",
        "ur3e",
        "ur5",
        "ur5e",
        "ur7e",
        "ur10",
        "ur10e",
        "ur12e",
        "ur15",
        "ur16e",
        "ur20",
        "ur30"
    ]
    for name in ur_names:
        urdf_file = os.path.join(urdf_root, f"{name}.urdf")
        yaml_dir = os.path.join(config_root, name)
        output_dir = os.path.join(submodel_root, name)

        if not os.path.exists(urdf_file):
            print(f"❌ 未找到 URDF: {urdf_file}")
            continue
        if not os.path.exists(yaml_dir):
            print(f"⚠️ YAML 配置缺失: {yaml_dir}（将使用空默认值）")
            os.makedirs(yaml_dir, exist_ok=True)

        try:
            main(urdf_file, yaml_dir, output_dir)
        except Exception as e:
            print(f"❌ 处理 {name} 时出错: {e}")
