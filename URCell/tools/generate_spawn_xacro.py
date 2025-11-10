"""
=============================================
 generate_spawn_xacro.py
---------------------------------------------
功能：
  从指定机器人型号的 AAS JSON 文件中生成单个机器人 spawn.xacro 文件，
  用于 ROS / Gazebo 仿真或数字孪生实例化。

支持：
  ✅ 自动清理 Link_/Joint_ 等前缀
  ✅ 从 AAS JSON 恢复真实 joint 类型
  ✅ 自动生成 axis / limit / dynamics
  ✅ 针对 igus_rebel 补全默认 limit
  ✅ 可选生成 transmission（--with-transmission）
  ✅ 可选控制接口类型（--interface position/velocity/effort）
  ✅ 自动插入 Gazebo ros_control 插件占位
  ✅ **新增：实例模式 (--instance-only)** —— 仅生成引用与位姿，不展开关节结构

---------------------------------------------
📘 使用说明：

1️⃣ 生成标准 UR5 机器人 spawn 文件（包含控制接口）：
    python generate_spawn_xacro.py --robot ur5 --with-transmission

2️⃣ 生成 igus ReBeL（含默认 limit、力矩控制接口）：
    python generate_spawn_xacro.py --robot igus_rebel --with-transmission --interface effort

3️⃣ 仅生成结构，不带控制接口：
    python generate_spawn_xacro.py --robot ur5

4️⃣ **生成实例模式（供 multi 使用，仅引用 + 根位姿）：**
    python generate_spawn_xacro.py --robot ur5 --instance-only

文件路径（按项目结构）：
  输入:  URCell/types/submodel/<robot>/*environment.json
  输出:  URCell/projects/spawns/<robot>_spawn.xacro
=============================================
"""

import json
import os
import re
import xml.etree.ElementTree as ET
import argparse
from datetime import datetime


def indent(elem, level=0):
    """格式化 XML 输出"""
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


def clean_prefix(name: str):
    """去掉 URx_Link_ / Link_ / Joint_ 前缀"""
    return re.sub(r"^(UR\d+[A-Za-z]*_)?(Link_|Joint_)", "", name)


def parse_literal(value):
    """把 JSON 中字符串化的列表/字典安全转为对象"""
    if isinstance(value, (dict, list, tuple)):
        return value
    if isinstance(value, str):
        try:
            return eval(value, {"__builtins__": {}})
        except Exception:
            return value
    return value


def add_transmission_block(robot_elem, joint_name, interface="position"):
    """添加标准 transmission 块"""
    trans = ET.SubElement(robot_elem, "transmission", {"name": f"{joint_name}_trans"})
    ET.SubElement(trans, "type").text = "transmission_interface/SimpleTransmission"

    joint_elem = ET.SubElement(trans, "joint", {"name": joint_name})
    ET.SubElement(joint_elem, "hardwareInterface").text = f"hardware_interface/{interface.capitalize()}JointInterface"

    act = ET.SubElement(trans, "actuator", {"name": f"{joint_name}_motor"})
    ET.SubElement(act, "hardwareInterface").text = f"hardware_interface/{interface.capitalize()}JointInterface"
    ET.SubElement(act, "mechanicalReduction").text = "1"


def generate_spawn_xacro(robot_type, json_data, output_path,
                         with_transmission=False, interface="position",
                         instance_only=False):
    """主逻辑"""
    robot_name = robot_type.upper()
    robot = ET.Element("robot", {"name": robot_name, "xmlns:xacro": "http://wiki.ros.org/xacro"})

    # 注释信息
    robot.append(ET.Comment(f"自动生成的 spawn 文件"))
    robot.append(ET.Comment(f"机器人类型: {robot_name}"))
    robot.append(ET.Comment(f"生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}"))

    # 引用原始 xacro
    include_path = f"${{urcell_root}}/types/robots/{robot_type}/{robot_type}.xacro"
    ET.SubElement(robot, "xacro:include", {"filename": include_path})

    # ============ 🧩 实例模式分支 ============
    if instance_only:
        robot.append(ET.Comment("实例模式：仅引用原始模型并定义根位姿"))
        ET.SubElement(robot, "xacro:" + robot_type, {
            "prefix": f"{robot_type}_1_",
            "base_xyz": "0 0 0",
            "base_rpy": "0 0 0"
        })
        link_world = ET.SubElement(robot, "link", {"name": "world"})
        joint_world = ET.SubElement(robot, "joint", {
            "name": f"world_to_{robot_type}_1",
            "type": "fixed"
        })
        ET.SubElement(joint_world, "parent", {"link": "world"})
        ET.SubElement(joint_world, "child", {"link": f"{robot_type}_1_base_link"})
        ET.SubElement(joint_world, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        indent(robot)
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        ET.ElementTree(robot).write(output_path, encoding="utf-8", xml_declaration=True)
        print(f"✅ 已生成实例化 spawn 文件（仅引用+位姿）: {output_path}")
        return
    # =========================================

    # Rebel 默认 limit
    rebel_limits = {
        "joint_1": {"lower": -2.356, "upper": 2.356, "velocity": 1.57, "effort": 10},
        "joint_2": {"lower": -2.094, "upper": 2.094, "velocity": 1.57, "effort": 8},
        "joint_3": {"lower": -2.094, "upper": 2.094, "velocity": 1.57, "effort": 8},
        "joint_4": {"lower": -3.141, "upper": 3.141, "velocity": 1.57, "effort": 5},
        "joint_5": {"lower": -2.094, "upper": 2.094, "velocity": 1.57, "effort": 5},
        "joint_6": {"lower": -3.141, "upper": 3.141, "velocity": 1.57, "effort": 5},
    }

    actuated_joints = []

    # 正常模式：从 JSON 解析 link/joint
    for submodel in json_data.get("submodels", []):
        for element in submodel.get("submodelElements", []):
            if "value" not in element:
                continue

            # 处理 Link
            if element["idShort"].startswith(("Link", "UR")):
                link_name = clean_prefix(element["idShort"])
                if not link_name:
                    continue
                ET.SubElement(robot, "link", {"name": link_name})

            # 处理 Joint
            elif element["idShort"].startswith(("Joint", "UR")):
                joint_name = clean_prefix(element["idShort"])
                parent = child = ""
                origin = axis = limit = dynamics = joint_type = None

                for prop in element["value"]:
                    pid, val = prop.get("idShort"), prop.get("value")
                    if pid == "parent":
                        parent = clean_prefix(val)
                    elif pid == "child":
                        child = clean_prefix(val)
                    elif pid == "origin":
                        origin = val
                    elif pid == "axis":
                        axis = val
                    elif pid == "limit":
                        limit = val
                    elif pid == "dynamics":
                        dynamics = val
                    elif pid == "type":
                        joint_type = str(val).lower() if val else None

                # 类型回退
                allowed = {"revolute", "continuous", "prismatic", "fixed", "planar", "floating"}
                if not joint_type or joint_type not in allowed:
                    joint_type = "fixed" if "fixed" in joint_name else "revolute"

                if not parent or not child:
                    continue

                joint_elem = ET.SubElement(robot, "joint", {"name": joint_name, "type": joint_type})
                ET.SubElement(joint_elem, "parent", {"link": parent})
                ET.SubElement(joint_elem, "child", {"link": child})

                # origin（如有错误，则默认 0）
                if origin:
                    try:
                        data = parse_literal(origin)
                        if isinstance(data, dict):
                            xyz = " ".join(map(str, data.get("xyz", [0, 0, 0])[:3]))
                            rpy = " ".join(map(str, data.get("rpy", [0, 0, 0])[:3]))
                        else:
                            xyz, rpy = "0 0 0", "0 0 0"
                        ET.SubElement(joint_elem, "origin", {"xyz": xyz, "rpy": rpy})
                    except Exception as e:
                        print(f"⚠️ origin 解析失败: {origin}, 错误: {e}")

                # 非 fixed 才加 axis / limit / dynamics
                if joint_type != "fixed":
                    if axis:
                        try:
                            axis_val = parse_literal(axis)
                            if isinstance(axis_val, (list, tuple)) and len(axis_val) >= 3:
                                ET.SubElement(joint_elem, "axis", {"xyz": " ".join(map(str, axis_val[:3]))})
                        except Exception as e:
                            print(f"⚠️ axis 解析失败: {axis}, 错误: {e}")

                    # limit
                    attrs = {}
                    if limit:
                        try:
                            lim = parse_literal(limit)
                            if isinstance(lim, dict):
                                for k in ["lower", "upper", "effort", "velocity"]:
                                    if lim.get(k) is not None:
                                        attrs[k] = str(lim[k])
                        except Exception:
                            pass

                    if robot_type.lower() == "igus_rebel" and not attrs:
                        default = rebel_limits.get(joint_name)
                        if default:
                            attrs = {k: str(v) for k, v in default.items()}

                    if attrs:
                        ET.SubElement(joint_elem, "limit", attrs)

                    if dynamics:
                        try:
                            dyn = parse_literal(dynamics)
                            if isinstance(dyn, dict):
                                d_attr = {k: str(v) for k, v in dyn.items() if v is not None}
                                if d_attr:
                                    ET.SubElement(joint_elem, "dynamics", d_attr)
                        except Exception:
                            pass

                    actuated_joints.append(joint_name)

    # 生成 transmission（若开启）
    if with_transmission:
        robot.append(ET.Comment("以下 transmission 为自动生成，用于 ROS 控制层"))
        for j in actuated_joints:
            add_transmission_block(robot, j, interface)

    # Gazebo 插件（ros_control）
    gz = ET.SubElement(robot, "gazebo")
    ET.SubElement(gz, "plugin", {"name": "ros_control", "filename": "libgazebo_ros_control.so"})

    indent(robot)
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    ET.ElementTree(robot).write(output_path, encoding="utf-8", xml_declaration=True)
    print(f"✅ 已生成 spawn 文件: {output_path}")


def main():
    parser = argparse.ArgumentParser(description="生成单个机器人 spawn.xacro 文件")
    parser.add_argument("--robot", required=True, help="机器人型号，例如 ur5 或 igus_rebel")
    parser.add_argument("--with-transmission", action="store_true", help="是否生成 transmission 块")
    parser.add_argument("--interface", default="position", choices=["position", "velocity", "effort"], help="控制接口类型")
    parser.add_argument("--instance-only", action="store_true", help="仅生成引用+根位姿（供 multi 使用）")
    args = parser.parse_args()

    robot_type = args.robot.lower()
    prefix = r"C:\Users\benhe\Desktop\abschlussarbeit\URCell"

    submodel_dir = os.path.join(prefix, "types", "submodel", robot_type)
    json_candidates = [f for f in os.listdir(submodel_dir) if f.endswith("environment.json")]
    if not json_candidates:
        print(f"❌ 未找到 {robot_type} 的 environment.json 文件")
        return

    input_json = os.path.join(submodel_dir, json_candidates[0])
    output_xacro = os.path.join(prefix, "projects", "spawns", f"{robot_type}_spawn.xacro")

    with open(input_json, "r", encoding="utf-8") as f:
        json_data = json.load(f)

    generate_spawn_xacro(robot_type, json_data, output_xacro,
                         args.with_transmission, args.interface,
                         args.instance_only)


if __name__ == "__main__":
    main()

"""
---------------------------------------------
📘 使用示例：

# 1️⃣ UR5 位置控制模式（带 transmission）
python generate_spawn_xacro.py --robot ur5 --with-transmission --interface position

# 2️⃣ igus Rebel 力矩控制模式
python generate_spawn_xacro.py --robot igus_rebel --with-transmission --interface effort

# 3️⃣ 仅生成结构（不含控制）
python generate_spawn_xacro.py --robot ur5

# 4️⃣ 生成实例模式（用于 multi 场景组合）
python generate_spawn_xacro.py --robot ur5 --instance-only

输出文件：
  URCell/projects/spawns/<robot>_spawn.xacro
---------------------------------------------
"""

