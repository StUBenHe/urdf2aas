#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Generate AAS Environment JSON (UR example style: string modelType + xs:* valueType)
Author: ChatGPT (aligned to user's ur3_environment.json schema)
"""

import json, os, uuid, xml.etree.ElementTree as ET
from pathlib import PurePosixPath

# --------- 路径配置 ----------
URDF_PATH = r"C:\Users\benhe\Desktop\abschlussarbeit\URCell\types\igus_rebel_description_ros2\rebel.urdf"
OUTPUT_DIR = r"C:\Users\benhe\Desktop\abschlussarbeit\URCell\types\submodel\igus_rebel"
OUTPUT_FILE = os.path.join(OUTPUT_DIR, "igus_rebel_environment.json")
# -----------------------------

def urn_aas(robot_up: str) -> str:
    return f"urn:rob:igus:{robot_up}:2025"

def urn_submodel(kind: str, robot_up: str, suffix: str = "safe") -> str:
    # 与示例保持一致：structure 使用 :safe:full 其余用 :safe
    if kind == "structure":
        return f"urn:submodel:robot:structure:{robot_up}:{suffix}:full"
    m = {
        "kinematics": "urn:submodel:robot:kinematics",
        "dynamics":   "urn:submodel:robot:dynamics",
        "control":    "urn:submodel:robot:control",
        "safety":     "urn:submodel:robot:safety",
        "viz":        "urn:submodel:robot:viz",
    }[kind]
    return f"{m}:{robot_up}:{suffix}"

def vt_str() -> str:
    return "xs:string"

def vt_dbl() -> str:
    return "xs:double"

def prop_str(id_short: str, value: str):
    return {"modelType": "Property", "idShort": id_short, "value": str(value), "valueType": vt_str()}

def prop_dbl(id_short: str, value):
    return {"modelType": "Property", "idShort": id_short, "value": str(value), "valueType": vt_dbl()}

def coll(id_short: str, elements: list):
    # 与示例一致：value 为数组
    return {"modelType": "SubmodelElementCollection", "idShort": id_short, "value": elements}

def file_element(id_short: str, path: str):
    # 根据扩展名给出 contentType，默认按 .dae 与 .stl
    p = path.strip()
    ext = p.split(".")[-1].lower() if "." in p else ""
    if ext == "dae":
        ct = "model/vnd.collada+xml"
    elif ext == "stl":
        ct = "model/stl"
    else:
        ct = "application/octet-stream"
    # 将 Windows/绝对地址压成类似示例的相对 POSIX 路径（如果你想，也可直接保留原串）
    norm = p
    if "file://" in p:
        # 只保留尾部相对片段，避免 AASX 报路径异常
        try:
            tail = p.split("/meshes/", 1)[-1]
            norm = str(PurePosixPath("/description/meshes") / tail)
        except Exception:
            norm = p
    return {"modelType": "File", "idShort": id_short, "contentType": ct, "value": norm}

def parse_vec3(s: str, as_rpy=False):
    # 输入形如 "x y z" -> 返回 [x, y, z] (double)
    try:
        xs, ys, zs = (float(t) for t in s.replace(",", " ").split())
    except Exception:
        xs, ys, zs = 0.0, 0.0, 0.0
    if as_rpy:
        return [xs, ys, zs]  # r, p, y
    return [xs, ys, zs]      # x, y, z

def build_links(root):
    links_out = []
    for l in root.findall("link"):
        lname = l.attrib.get("name", "")
        items = [prop_str("name", lname)]

        inertial = l.find("inertial")
        if inertial is not None:
            inert_val = []
            mass = inertial.find("mass")
            if mass is not None and "value" in mass.attrib:
                inert_val.append(prop_dbl("mass", mass.attrib["value"]))

            origin = inertial.find("origin")
            if origin is not None and "xyz" in origin.attrib:
                x, y, z = parse_vec3(origin.attrib.get("xyz", "0 0 0"))
                inert_val.append(coll("com", [prop_dbl("x", x), prop_dbl("y", y), prop_dbl("z", z)]))

            inertia = inertial.find("inertia")
            if inertia is not None:
                inz = []
                for k in ["ixx", "iyy", "izz", "ixy", "ixz", "iyz"]:
                    if k in inertia.attrib:
                        inz.append(prop_dbl(k, inertia.attrib[k]))
                if inz:
                    inert_val.append(coll("inertia", inz))

            if inert_val:
                items.append(coll("inertial", inert_val))

        # visuals
        visuals = l.findall("visual")
        if visuals:
            vitems = []
            for i, v in enumerate(visuals):
                mesh = v.find(".//mesh")
                if mesh is not None and "filename" in mesh.attrib:
                    vitems.append(coll(f"visual_{i}", [file_element("mesh", mesh.attrib["filename"])]))
            if vitems:
                items.append(coll("visuals", vitems))

        # collisions
        collisions = l.findall("collision")
        if collisions:
            citems = []
            for i, c in enumerate(collisions):
                mesh = c.find(".//mesh")
                if mesh is not None and "filename" in mesh.attrib:
                    citems.append(coll(f"collision_{i}", [file_element("mesh", mesh.attrib["filename"])]))
            if citems:
                items.append(coll("collisions", citems))

        links_out.append(coll(f"Link_{lname}", items))
    return links_out

def build_joints(root):
    joints_out = []
    for j in root.findall("joint"):
        jname = j.attrib.get("name", "")
        jtype = j.attrib.get("type", "fixed")
        parent = j.find("parent")
        child = j.find("child")
        items = [
            prop_str("type", jtype),
            prop_str("parent", parent.attrib.get("link", "")) if parent is not None else prop_str("parent", ""),
            prop_str("child", child.attrib.get("link", "")) if child is not None else prop_str("child", "")
        ]

        origin = j.find("origin")
        if origin is not None:
            xyz = parse_vec3(origin.attrib.get("xyz", "0 0 0"))
            rpy = parse_vec3(origin.attrib.get("rpy", "0 0 0"), as_rpy=True)
            items.append(
                coll("origin", [
                    coll("xyz", [prop_dbl("x", xyz[0]), prop_dbl("y", xyz[1]), prop_dbl("z", xyz[2])]),
                    coll("rpy", [prop_dbl("r", rpy[0]), prop_dbl("p", rpy[1]), prop_dbl("y", rpy[2])]),
                ])
            )

        axis = j.find("axis")
        if axis is not None:
            ax = parse_vec3(axis.attrib.get("xyz", "0 0 1"))
            items.append(
                coll("axis", [prop_dbl("x", ax[0]), prop_dbl("y", ax[1]), prop_dbl("z", ax[2])])
            )

        limit = j.find("limit")
        if limit is not None and limit.attrib:
            lims = []
            for k in ["lower", "upper", "velocity", "effort"]:
                if k in limit.attrib:
                    lims.append(prop_dbl(k, limit.attrib[k]))
            if lims:
                items.append(coll("limits", lims))

        # dynamics (如果需要，可从 URDF 解析 <dynamics> damping/friction)
        dyn = j.find("dynamics")
        if dyn is not None:
            ditems = []
            if "damping" in dyn.attrib:
                ditems.append(prop_dbl("damping", dyn.attrib["damping"]))
            if "friction" in dyn.attrib:
                ditems.append(prop_dbl("friction", dyn.attrib["friction"]))
            if ditems:
                items.append(coll("dynamics", ditems))

        joints_out.append(coll(f"Joint_{jname}", items))

    return joints_out

def generate_structure_submodel(robot_up: str, root) -> dict:
    submodel = {
        "modelType": "Submodel",
        "id": urn_submodel("structure", robot_up),
        "idShort": f"{robot_up}_StructureSafe",
        "category": f"Robot:{robot_up}",
        "submodelElements": []
    }
    links = build_links(root)
    joints = build_joints(root)
    if links:
        submodel["submodelElements"].append(coll("Links", links))
    if joints:
        submodel["submodelElements"].append(coll("Joints", joints))
    # 可选：Transmissions（你的示例里有；若 URDF 无 <transmission> 就不生成）
    return submodel

def generate_empty_submodel(id_urn: str, id_short: str, category: str) -> dict:
    return {"modelType": "Submodel", "id": id_urn, "idShort": id_short, "category": category, "submodelElements": []}

def main():
    print("🔧 Generating AAS JSON (UR-example style) ...")
    if not os.path.exists(URDF_PATH):
        print(f"❌ URDF not found: {URDF_PATH}")
        return
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)

    tree = ET.parse(URDF_PATH)
    root = tree.getroot()
    robot_name = root.attrib.get("name", "robot").strip()
    robot_up = robot_name.upper()

    # AAS
    shell_id = urn_aas(robot_up)
    aas = {
        "id": shell_id,
        "idShort": robot_up,
        "assetInformation": {
            "globalAssetId": f"urn:rob:igus:{robot_up}",
            "kind": "Template",
            "assetKind": "Template"
        },
        "submodels": []
    }

    # Submodels
    sm_structure = generate_structure_submodel(robot_up, root)
    sm_kin = generate_empty_submodel(urn_submodel("kinematics", robot_up), f"{robot_up}_KinematicsSafe", f"Robot:{robot_up}")
    sm_dyn = generate_empty_submodel(urn_submodel("dynamics", robot_up),  f"{robot_up}_DynamicsSafe",   f"Robot:{robot_up}")
    sm_ctl = generate_empty_submodel(urn_submodel("control", robot_up),   f"{robot_up}_ControlSafe",    f"Robot:{robot_up}")
    sm_saf = generate_empty_submodel(urn_submodel("safety", robot_up),    f"{robot_up}_SafetySafe",     f"Robot:{robot_up}")
    sm_viz = generate_empty_submodel(urn_submodel("viz", robot_up),       f"{robot_up}_VizSafe",        f"Robot:{robot_up}")

    # 引用顺序与示例一致
    for sm in [sm_structure, sm_kin, sm_dyn, sm_ctl, sm_saf, sm_viz]:
        aas["submodels"].append({"type": "ModelReference", "keys": [{"type": "Submodel", "value": sm["id"]}]})

    env = {
        "assetAdministrationShells": [aas],
        "submodels": [sm_structure, sm_kin, sm_dyn, sm_ctl, sm_saf, sm_viz]
    }

    with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
        json.dump(env, f, indent=2, ensure_ascii=False)

    print(f"✅ Output: {OUTPUT_FILE}")
    print("👉 打开 AASX Package Explorer → File → Import JSON 导入即可")

if __name__ == "__main__":
    main()
