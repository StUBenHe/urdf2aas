import os
import json
import math


# ---------------------------------------
# 工具函数
# ---------------------------------------

def parse_json_str(value):
    """解析 JSON 字符串为 dict"""
    if isinstance(value, dict):
        return value
    if isinstance(value, str):
        try:
            return json.loads(value)
        except:
            return None
    return None


def matrix_to_rpy(mat):
    """将 3×3 惯性坐标旋转矩阵转换为 rpy (XYZ convention)"""
    r11, r12, r13 = mat[0]
    r21, r22, r23 = mat[1]
    r31, r32, r33 = mat[2]

    if abs(r31) != 1:
        pitch = -math.asin(r31)
        roll = math.atan2(r32 / math.cos(pitch), r33 / math.cos(pitch))
        yaw = math.atan2(r21 / math.cos(pitch), r11 / math.cos(pitch))
    else:
        yaw = 0
        if r31 == -1:
            pitch = math.pi / 2
            roll = yaw + math.atan2(r12, r13)
        else:
            pitch = -math.pi / 2
            roll = -yaw + math.atan2(-r12, -r13)

    return [roll, pitch, yaw]


def make_origin(origin_json):
    """从 JSON origin 生成 xacro origin"""
    if not origin_json:
        return '<origin xyz="0 0 0" rpy="0 0 0"/>'

    xyz = origin_json.get("xyz", ["0", "0", "0"])
    rpy = origin_json.get("rpy", ["0", "0", "0"])

    return f'<origin xyz="{xyz[0]} {xyz[1]} {xyz[2]}" rpy="{rpy[0]} {rpy[1]} {rpy[2]}"/>'


def extract_robot_type_from_mesh(mesh_path):
    """从 mesh path 中自动提取 robot_type（如 ur5）"""
    parts = mesh_path.replace("\\", "/").split("/")
    for p in parts:
        if p.startswith("ur") or p.startswith("igus"):
            return p  # ur5 / ur3 / igus_rebel_6dof
    return "unknown"


# ---------------------------------------
# 主函数：生成结构 XACRO
# ---------------------------------------

def generate_xacro_structure(env_json_path, output_path):
    """从 AAS Environment JSON 自动生成 XACRO"""

    with open(env_json_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    structure = []
    prefix = "${prefix}"
    mesh_root = "${mesh_root}"

    # 读取 3 个 Submodel
    SM = {}
    for sm in data["submodels"]:
        sid = sm["idShort"]
        SM[sid] = sm

    StructureSM = next(s for s in data["submodels"] if "StructureSubmodel" in s["idShort"])
    VisualizationSM = next(s for s in data["submodels"] if "VisualizationSubmodel" in s["idShort"])
    DynamicsSM = next(s for s in data["submodels"] if "DynamicsSubmodel" in s["idShort"])

    # 建立可快速索引的结构
    VisDict = {v["idShort"]: v for v in VisualizationSM["submodelElements"]}
    DynDict = {v["idShort"]: v for v in DynamicsSM["submodelElements"]}

    # ========================================
    # 写入 header
    # ========================================
    structure.append('<?xml version="1.0"?>')
    structure.append(f"<!-- Auto Generated from {os.path.basename(env_json_path)} -->\n")
    structure.append('<robot xmlns:xacro="http://ros.org/wiki/xacro" name="auto_robot">')

    # ========================================
    # 固定生成顺序（完全匹配模板）
    # ========================================

    link_order = [
        "Link_base_link",
        "Link_base_link_inertia",
        "Link_shoulder_link",
        "Link_upper_arm_link",
        "Link_forearm_link",
        "Link_wrist_1_link",
        "Link_wrist_2_link",
        "Link_wrist_3_link",
        "Link_base",
        "Link_flange",
        "Link_tool0"
    ]

    # ========================================
    # 遍历 link
    # ========================================

    for link_key in link_order:
        link_name = link_key.replace("Link_", "")

        elem = next((e for e in StructureSM["submodelElements"] if e["idShort"] == link_key), None)

        structure.append(f'\n  <link name="{prefix}_{link_name}">')

        if elem:
            # ------------------------
            # inertial (来自 Dynamics)
            # ------------------------
            if link_key in DynDict:
                dyn = DynDict[link_key]

                mass = None
                inertia_mat = None
                inertia_origin = None

                for v in dyn["value"]:
                    if v["idShort"] == "mass":
                        mass = v.get("value")
                    elif v["idShort"] == "inertia_matrix":
                        inertia_mat = parse_json_str(v.get("value"))
                    elif v["idShort"] == "origin":
                        inertia_origin = parse_json_str(v.get("value"))

                if mass:
                    structure.append("    <inertial>")
                    structure.append(f'      <mass value="{mass}"/>')

                    # inertia matrix 转换
                    if inertia_mat:
                        i = inertia_mat
                        structure.append(
                            f'      <inertia ixx="{i[0][0]}" ixy="{i[0][1]}" ixz="{i[0][2]}" '
                            f'iyy="{i[1][1]}" iyz="{i[1][2]}" izz="{i[2][2]}"/>'
                        )
                    else:
                        structure.append(
                            '      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>'
                        )

                    if inertia_origin:
                        rpy = matrix_to_rpy(inertia_origin["rpy_matrix"]) if "rpy_matrix" in inertia_origin else [0,0,0]
                        xyz = inertia_origin.get("xyz", ["0","0","0"])
                        structure.append(
                            f'      <origin xyz="{xyz[0]} {xyz[1]} {xyz[2]}" '
                            f'rpy="{rpy[0]} {rpy[1]} {rpy[2]}"/>'
                        )

                    structure.append("    </inertial>")

            # ------------------------
            # visual/collision 来自 VisualizationSM
            # ------------------------
            if link_key in VisDict:
                vis_group = VisDict[link_key]

                for v in vis_group["value"]:
                    vtype = v["idShort"]  # visual 或 collision
                    props = {p["idShort"]: p.get("value") for p in v["value"]}
                    mesh_path = props.get("mesh")

                    if mesh_path:
                        robot_type = extract_robot_type_from_mesh(mesh_path)
                        filename = os.path.basename(mesh_path)

                        structure.append(f"    <{vtype}>")
                        structure.append(
                            f'      <geometry><mesh filename="file://{mesh_root}/{robot_type}/{vtype}/{filename}"/></geometry>'
                        )

                        origin = parse_json_str(props.get("origin"))
                        structure.append(f"      {make_origin(origin)}")

                        # visual 加材质
                        if vtype == "visual":
                            structure.append(
                                '      <material name="LightGrey"><color rgba="0.7 0.7 0.7 1.0" /></material>'
                            )

                        structure.append(f"    </{vtype}>")

        structure.append("  </link>")

    # ========================================
    # JOINTS
    # ========================================

    for elem in StructureSM["submodelElements"]:
        if not elem["idShort"].startswith("Joint_"):
            continue

        jname = elem["idShort"].replace("Joint_", "")
        props = {v["idShort"]: v.get("value") for v in elem["value"]}

        joint_type = props.get("type", "fixed")
        parent = props.get("parent")
        child = props.get("child")
        origin = parse_json_str(props.get("origin"))

        structure.append(f'\n  <joint name="{prefix}_{jname}" type="{joint_type}">')
        structure.append(f'    <parent link="{prefix}_{parent}"/>')
        structure.append(f'    <child link="{prefix}_{child}"/>')
        structure.append(f"    {make_origin(origin)}")

        if joint_type == "revolute":
            structure.append('    <axis xyz="0 0 1"/>')

        structure.append("  </joint>")

    structure.append("\n</robot>\n")

    # 写入文件
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        f.write("\n".join(structure))

    print(f"✔ 完成：生成 XACRO → {output_path}")


if __name__ == "__main__":
    print(">>> Running UR Structure Generator")

    json_path = r"C:\Users\benhe\Desktop\abschlussarbeit\URCell\types\submodel\ur5\ur5_environment.json"
    output_path = r"C:\Users\benhe\Desktop\abschlussarbeit\URCell\projects\ur5_spawn.xacro"

    generate_xacro_structure(json_path, output_path)
