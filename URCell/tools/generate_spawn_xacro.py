"""
generate_spawn_xacro.py — full expansion with links + transmissions
-------------------------------------------------------------------
从 environment.json 生成完整 spawn 宏文件：
✅ 包含所有 joint
✅ 自动创建 link 节点
✅ 自动生成 transmission 块
✅ 输出 <xacro:macro> 格式，可直接 include 于 multi_ur.xacro
"""

import json
import os
import xml.etree.ElementTree as ET
import argparse
from datetime import datetime


def indent(elem, level=0):
    """美化 XML 输出"""
    i = "\n" + level * "  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        for subelem in elem:
            indent(subelem, level + 1)
        if not subelem.tail or not subelem.tail.strip():
            subelem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i


def parse_literal(value):
    """解析 JSON 字符串为字典/列表"""
    if isinstance(value, (dict, list)):
        return value
    if isinstance(value, str):
        try:
            return json.loads(value)
        except Exception:
            return value
    return value


def add_transmission(robot_elem, joint_name):
    """为每个关节添加 transmission 块"""
    trans = ET.SubElement(robot_elem, "transmission", {"name": f"{joint_name}_trans"})
    ET.SubElement(trans, "type").text = "transmission_interface/SimpleTransmission"

    j_elem = ET.SubElement(trans, "joint", {"name": joint_name})
    ET.SubElement(j_elem, "hardwareInterface").text = "hardware_interface/PositionJointInterface"

    a_elem = ET.SubElement(trans, "actuator", {"name": f"{joint_name}_motor"})
    ET.SubElement(a_elem, "hardwareInterface").text = "hardware_interface/PositionJointInterface"
    ET.SubElement(a_elem, "mechanicalReduction").text = "1"


def generate_macro(robot_type, json_path, output_path):
    """主生成函数"""
    robot_name = robot_type.upper()
    robot = ET.Element("robot", {"name": robot_name, "xmlns:xacro": "http://wiki.ros.org/xacro"})

    robot.append(ET.Comment("Auto-generated full macro (with links + transmissions)"))
    robot.append(ET.Comment(f"Robot: {robot_name}"))
    robot.append(ET.Comment(f"Generated at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}"))

    macro = ET.SubElement(robot, "xacro:macro", {
        "name": f"{robot_type.split('_')[0]}_spawn",
        "params": "ns xyz rpy"
    })

    # world_joint
    joint_world = ET.SubElement(macro, "joint", {"name": "${ns}_world_joint", "type": "fixed"})
    ET.SubElement(joint_world, "parent", {"link": "world"})
    ET.SubElement(joint_world, "child", {"link": "${ns}_base_link"})
    ET.SubElement(joint_world, "origin", {"xyz": "${xyz}", "rpy": "${rpy}"})

    # link 集合
    all_links = {"world", "${ns}_base_link"}
    all_joints = []  # 用于 transmission

    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    for submodel in data.get("submodels", []):
        for elem in submodel.get("submodelElements", []):
            if "value" not in elem:
                continue
            name = elem.get("idShort", "")
            if not name:
                continue

            props = {e["idShort"]: e.get("value") for e in elem["value"] if isinstance(e, dict)}
            parent = props.get("parent", "")
            prefit = props.get("prefit", "")
            origin = parse_literal(props.get("origin", {}))
            limit = parse_literal(props.get("limit", {}))
            variable = props.get("variable", None)

            # joint
            joint_full_name = f"${{ns}}_{name}"
            joint = ET.SubElement(macro, "joint", {"name": joint_full_name, "type": "revolute"})

            if parent:
                ET.SubElement(joint, "parent", {"link": f"${{ns}}_{parent}"})
                all_links.add(f"${{ns}}_{parent}")
            if prefit:
                ET.SubElement(joint, "child", {"link": f"${{ns}}_{prefit}"})
                all_links.add(f"${{ns}}_{prefit}")

            if isinstance(origin, dict):
                xyz = " ".join(map(str, origin.get("xyz", [0, 0, 0])))
                rpy = " ".join(map(str, origin.get("rpy", [0, 0, 0])))
                ET.SubElement(joint, "origin", {"xyz": xyz, "rpy": rpy})

            if isinstance(limit, dict) and any(limit.values()):
                ET.SubElement(joint, "limit", {k: str(v) for k, v in limit.items() if v is not None})

            if variable is not None:
                ET.SubElement(joint, "xacro:property", {"name": f"{name}_var"}).text = str(variable)

            all_joints.append(joint_full_name)

    # 添加 link 定义（在宏开头）
    link_block = [ET.Element("link", {"name": link}) for link in sorted(all_links)]
    macro[:] = link_block + macro[:]

    # 添加 transmission 块（宏外）
    robot.append(ET.Comment("Auto-generated transmission blocks"))
    for j in all_joints:
        add_transmission(robot, j)

    indent(robot)
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    ET.ElementTree(robot).write(output_path, encoding="utf-8", xml_declaration=True)

    print(f"✅ 生成完成: {output_path}")
    print(f"   链接数: {len(all_links)}, 关节数: {len(all_joints)}, Transmission数: {len(all_joints)}")


def main():
    parser = argparse.ArgumentParser(description="Generate full spawn macro with links + transmissions from JSON")
    parser.add_argument("--robot", required=True, help="e.g. ur5 or igus_rebel_6dof")
    args = parser.parse_args()

    robot_key = args.robot.lower()
    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

    if robot_key.startswith("igus_rebel"):
        folder = "igus_rebel"
        json_filename = f"{robot_key}_environment.json"
        output_basename = "igus_rebel_spawn.xacro"
    else:
        folder = robot_key
        json_filename = f"{robot_key}_environment.json"
        output_basename = f"{robot_key}_spawn.xacro"

    json_path = os.path.join(base_dir, "types", "submodel", folder, json_filename)
    output_path = os.path.join(base_dir, "projects", "spawns", output_basename)

    if not os.path.exists(json_path):
        print(f"❌ 找不到 JSON 文件: {json_path}")
        return

    generate_macro(robot_key, json_path, output_path)


if __name__ == "__main__":
    main()

