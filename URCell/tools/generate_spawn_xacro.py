#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
generate_spawn_xacro_prefixed_multi.py
---------------------------------------------------
一键生成：
✅ 各机器人 *prefixed.xacro（带 tf_prefix）
✅ 对应 joint_limits.yaml（如不存在自动生成）
✅ 最终 multi_ur.xacro（自动 include 所有机器人）
"""

import os
import yaml
import json
import xml.etree.ElementTree as ET
from datetime import datetime

# ======================================================
# 工具函数
# ======================================================

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
    """解析 JSON 字符串为 dict/list"""
    if isinstance(value, (dict, list)):
        return value
    if isinstance(value, str):
        try:
            return json.loads(value)
        except Exception:
            return value
    return value


def load_joint_limits(robot_type, env_data, spawn_dir):
    """加载或自动生成 joint_limits.yaml"""
    limit_filename = os.path.join(spawn_dir, f"{robot_type}_joint_limits.yaml")

    if os.path.exists(limit_filename):
        print(f"🔧 使用外部 joint limits: {limit_filename}")
        with open(limit_filename, "r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}

    print(f"⚙️ 自动生成 joint limits: {limit_filename}")
    limits = {"joints": {}}

    for submodel in env_data.get("submodels", []):
        sid = submodel.get("idShort", "")
        if "Control" in sid or "Safety" in sid:
            for elem in submodel.get("submodelElements", []):
                jname = elem.get("idShort", "")
                if not jname:
                    continue
                props = {e["idShort"]: e.get("value") for e in elem.get("value", []) if isinstance(e, dict)}
                lower = props.get("lower")
                upper = props.get("upper")
                effort = props.get("effort")
                velocity = props.get("velocity")
                if any([lower, upper, effort, velocity]):
                    limits["joints"][jname] = {
                        "lower": float(lower) if lower else None,
                        "upper": float(upper) if upper else None,
                        "effort": float(effort) if effort else None,
                        "velocity": float(velocity) if velocity else None
                    }

    os.makedirs(spawn_dir, exist_ok=True)
    with open(limit_filename, "w", encoding="utf-8") as f:
        yaml.dump(limits, f, sort_keys=False, allow_unicode=True)
    return limits


# ======================================================
# 生成单个机器人 prefixed.xacro
# ======================================================

def generate_prefixed_xacro(robot_type, json_path, output_path):
    """从 JSON 生成完整带 tf_prefix 的 xacro"""
    print(f"🧩 生成 {robot_type}_prefixed.xacro ...")

    robot = ET.Element("robot", {
        "xmlns:xacro": "http://ros.org/wiki/xacro",
        "name": f"{robot_type}",
    })

    robot.append(ET.Comment(f"Auto-generated prefixed XACRO for {robot_type.upper()}"))
    robot.append(ET.Comment(f"Generated at {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}"))

    ET.SubElement(robot, "xacro:arg", {"name": "tf_prefix", "default": f"{robot_type}_"})
    ET.SubElement(robot, "xacro:property", {
        "name": "joint_limits",
        "value": f"${{{{load_yaml('$(find ur_description)/config/{robot_type}_joint_limits.yaml')}}}}"

    })

    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    limits = load_joint_limits(robot_type, data, os.path.dirname(output_path))

    links = set()
    joints = []

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
            jtype = props.get("type", "revolute")

            joint = ET.SubElement(robot, "joint", {
                "name": f"${{tf_prefix}}{name}",
                "type": jtype
            })
            if parent:
                ET.SubElement(joint, "parent", {"link": f"${{tf_prefix}}{parent}"})
                links.add(parent)
            if prefit:
                ET.SubElement(joint, "child", {"link": f"${{tf_prefix}}{prefit}"})
                links.add(prefit)
            if isinstance(origin, dict):
                xyz = " ".join(map(str, origin.get("xyz", [0, 0, 0])))
                rpy = " ".join(map(str, origin.get("rpy", [0, 0, 0])))
                ET.SubElement(joint, "origin", {"xyz": xyz, "rpy": rpy})

            # 限位
            limit_info = limits.get("joints", {}).get(name, None)
            if limit_info:
                limit_attrs = {k: str(v) for k, v in limit_info.items() if v is not None}
                if limit_attrs:
                    ET.SubElement(joint, "limit", limit_attrs)

            joints.append(name)

    for link in sorted(links):
        ET.SubElement(robot, "link", {"name": f"${{tf_prefix}}{link}"})

    indent(robot)
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    ET.ElementTree(robot).write(output_path, encoding="utf-8", xml_declaration=True)
    print(f"✅ 已生成 {output_path}")


# ======================================================
# 主入口：multi 生成
# ======================================================

def generate_multi_ur(multi_yaml):
    with open(multi_yaml, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    robots = cfg.get("robots", [])
    base_dir = os.path.dirname(os.path.abspath(multi_yaml))
    spawns_dir = os.path.join(base_dir, "spawns")
    submodel_dir = os.path.join(base_dir, "../types/submodel")

    os.makedirs(spawns_dir, exist_ok=True)

    # 生成各 prefixed 模型
    include_lines = []
    body_lines = []

    for robot_cfg in robots:
        rtype = robot_cfg.get("type", "").lower()
        rname = robot_cfg.get("name", rtype)
        xyz = " ".join(map(str, robot_cfg.get("xyz", [0, 0, 0])))
        rpy = " ".join(map(str, robot_cfg.get("rpy", [0, 0, 0])))
        enable = str(robot_cfg.get("enable", True)).lower()

        # 自动查找该子模型目录下任何 *_environment.json 文件
        sub_dir = os.path.join(submodel_dir, rtype)
        env_candidates = [
            os.path.join(sub_dir, f) for f in os.listdir(sub_dir)
            if f.endswith("_environment.json")
        ]

        if not env_candidates:
            print(f"❌ 未找到 {rtype} 的环境文件（*_environment.json）: {sub_dir}")
            continue

        # 使用找到的第一个匹配文件
        json_path = env_candidates[0]
        print(f"📄 使用环境文件: {json_path}")

        out_xacro = os.path.join(spawns_dir, f"{rtype}_prefixed.xacro")
        generate_prefixed_xacro(rtype, json_path, out_xacro)

        include_lines.append(f'  <xacro:include filename="$(arg urcell_root)/projects/spawns/{rtype}_prefixed.xacro"/>')
        body_lines.append(f"""
  <xacro:if value="{enable}">
    <joint name="world_to_{rname}" type="fixed">
      <parent link="world"/>
      <child link="${{{rname}_tf_prefix}}base_link"/>
      <origin xyz="{xyz}" rpy="{rpy}"/>
    </joint>
    <xacro:{rtype}_prefixed tf_prefix="${{{rname}_tf_prefix}}"/>
  </xacro:if>""")

    # 生成 multi_ur.xacro
    out_world = os.path.join(base_dir, "multi_ur.xacro")
    header = """<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="multi_ur">
  <xacro:arg name="urcell_root" default="../../../"/>
  <link name="world"/>
"""
    footer = "\n</robot>\n"

    with open(out_world, "w", encoding="utf-8") as f:
        f.write(header + "\n".join(include_lines) + "\n" + "\n".join(body_lines) + footer)

    print(f"🌍 已生成 multi_ur.xacro: {out_world}")


# ======================================================
# 执行入口
# ======================================================

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Auto generate prefixed xacro + multi_ur.xacro")
    parser.add_argument("--multi", required=True, help="multi_ur.yaml 配置路径")
    args = parser.parse_args()

    generate_multi_ur(args.multi)
