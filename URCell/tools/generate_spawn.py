import json
import yaml
from pathlib import Path

# ---------------------------------------------------------
# 工具函数
# ---------------------------------------------------------

def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)

def save_text(path, text):
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)

def save_yaml(path, data):
    with open(path, "w", encoding="utf-8") as f:
        yaml.dump(data, f, sort_keys=False)

def fix_mesh_path(original):
    """
    从 JSON 提供的路径自动转换为 ROS 可用的:
    package://ur_description/meshes/...
    """
    if not original:
        return ""

    # 1) 找到 meshes / 后面的所有内容
    idx = original.find("meshes")
    if idx == -1:
        raise ValueError(f"JSON mesh 路径格式不正确: {original}")

    sub = original[idx + len("meshes/"):]
    return f"package://ur_description/meshes/{sub}"

# ---------------------------------------------------------
# 解析 AAS 子模型
# ---------------------------------------------------------

def extract_structure_submodel(json_data):
    """读取 StructureSubmodel 中 link 的 visual / collision 信息"""
    struct = {}
    for sm in json_data["submodels"]:
        if sm["idShort"] == "StructureSubmodel":
            for element in sm["submodelElements"]:
                link_name = element["idShort"]
                struct[link_name] = {}

                # 子元素是 value 列表
                for sub in element["value"]:
                    if sub["idShort"].startswith("visual"):
                        # visual_0 / visual_1 ...
                        key = sub["idShort"]
                        struct[link_name][key] = {
                            "origin": sub["value"]["origin"],
                            "mesh": fix_mesh_path(sub["value"]["mesh"])
                        }
                    if sub["idShort"].startswith("collision"):
                        key = sub["idShort"]
                        struct[link_name][key] = {
                            "origin": sub["value"]["origin"],
                            "mesh": fix_mesh_path(sub["value"]["mesh"])
                        }
    return struct


def extract_dynamics_submodel(json_data):
    """读取 DynamicsSubmodel 中 inertia / mass / inertial origin 信息"""
    result = {}
    for sm in json_data["submodels"]:
        if sm["idShort"] == "DynamicsSubmodel":
            for link in sm["submodelElements"]:
                name = link["idShort"]
                result[name] = {}
                for sub in link["value"]:
                    if sub["idShort"] == "mass":
                        result[name]["mass"] = sub["value"]
                    if sub["idShort"] == "inertia_origin":
                        result[name]["inertia_origin"] = sub["value"]
                    if sub["idShort"] == "inertia":
                        result[name]["inertia"] = sub["value"]
    return result


def extract_kinematics_submodel(json_data):
    """读取 KinematicsSubmodel: joint parent/child/origin/type"""
    joints = {}
    for sm in json_data["submodels"]:
        if sm["idShort"] == "KinematicsSubmodel":
            for j in sm["submodelElements"]:
                jname = j["idShort"]
                joints[jname] = {}
                for sub in j["value"]:
                    if sub["idShort"] == "parent":
                        joints[jname]["parent"] = sub["value"]
                    elif sub["idShort"] == "prefit":  # child link
                        joints[jname]["child"] = sub["value"]
                    elif sub["idShort"] == "origin":
                        joints[jname]["origin_xyz"] = sub["value"]["xyz"]
                        joints[jname]["origin_rpy"] = sub["value"]["rpy"]
    return joints


def extract_control_submodel(json_data):
    """读取 ControlSubmodel: joint velocity/effort"""
    limits = {}

    for sm in json_data["submodels"]:
        if sm["idShort"] == "ControlSubmodel":
            for element in sm["submodelElements"]:
                jname = element["idShort"]
                limits[jname] = {}
                for sub in element["value"]:
                    if sub["idShort"] == "maxVelocity":
                        limits[jname]["max_velocity"] = sub["value"]
                    if sub["idShort"] == "effort":
                        limits[jname]["effort"] = sub["value"]
    return limits


def extract_safety_submodel(json_data):
    """读取 SafetySubmodel: joint upper/lower limit"""
    limits = {}
    for sm in json_data["submodels"]:
        if sm["idShort"] == "SafetySubmodel":
            for element in sm["submodelElements"]:
                jname = element["idShort"]
                limits[jname] = {}
                for sub in element["value"]:
                    if sub["idShort"] == "upper":
                        limits[jname]["upper"] = sub["value"]
                    if sub["idShort"] == "lower":
                        limits[jname]["lower"] = sub["value"]
    return limits

# ---------------------------------------------------------
# 渲染模板
# ---------------------------------------------------------

def fill_template(template_text, mapping):
    """简单 placeholder 替换"""
    out = template_text
    for key, value in mapping.items():
        out = out.replace("{{" + key + "}}", str(value))
    return out

# ---------------------------------------------------------
# 主流程：从 JSON → 模板 → XACRO
# ---------------------------------------------------------

def generate_spawn(json_path: str):
    # 读 JSON
    data = load_json(json_path)

    # 模板
    template_path = Path("ur_spawn_template.xacro")
    template_text = template_path.read_text(encoding="utf-8")

    # 提取子模型
    struct = extract_structure_submodel(data)
    dyn = extract_dynamics_submodel(data)
    kin = extract_kinematics_submodel(data)
    ctl = extract_control_submodel(data)
    safe = extract_safety_submodel(data)

    # 机器人名字
    robot_type = data["idShort"]  # 例如 "ur3_environment"
    robot_name = robot_type.replace("_environment", "_spawn_hand")

    # joint limits YAML 输出
    joint_limits_yaml = f"joint_limits_{robot_type.replace('_environment','')}.yaml"

    # 合并 joint limit 数据（Control + Safety）
    joint_limit_out = {"joint_limits": {}}
    for j in kin:
        if j not in joint_limit_out["joint_limits"]:
            joint_limit_out["joint_limits"][j] = {}
        if j in safe:
            joint_limit_out["joint_limits"][j]["min_position"] = safe[j].get("lower", 0)
            joint_limit_out["joint_limits"][j]["max_position"] = safe[j].get("upper", 0)
        if j in ctl:
            joint_limit_out["joint_limits"][j]["max_velocity"] = ctl[j].get("max_velocity", 1.0)

    save_yaml(joint_limit_yaml, joint_limit_out)

    # 最终 mapping 构建
    mapping = {
        "robot_name": robot_name,
        "joint_limit_yaml": joint_limit_yaml,
    }

    # -----------------------------------------------------
    # 填写 link 数据（这里只是一个示例，实际应循环填充）
    # -----------------------------------------------------

    for link in struct:
        base = link  # JSON idShort，如 "upper_arm_link"

        # visual_0
        if "visual_0" in struct[base]:
            mapping[f"{base}_visual_mesh"] = struct[base]["visual_0"]["mesh"]
            mapping[f"{base}_visual_xyz"] = " ".join(map(str, struct[base]["visual_0"]["origin"]["xyz"]))
            mapping[f"{base}_visual_rpy"] = " ".join(map(str, struct[base]["visual_0"]["origin"]["rpy"]))
        else:
            mapping[f"{base}_visual_mesh"] = ""
            mapping[f"{base}_visual_xyz"] = "0 0 0"
            mapping[f"{base}_visual_rpy"] = "0 0 0"

        # collision
        if "collision_0" in struct[base]:
            mapping[f"{base}_collision_mesh"] = struct[base]["collision_0"]["mesh"]
            mapping[f"{base}_collision_xyz"] = " ".join(map(str, struct[base]["collision_0"]["origin"]["xyz"]))
            mapping[f"{base}_collision_rpy"] = " ".join(map(str, struct[base]["collision_0"]["origin"]["rpy"]))
        else:
            mapping[f"{base}_collision_mesh"] = ""
            mapping[f"{base}_collision_xyz"] = "0 0 0"
            mapping[f"{base}_collision_rpy"] = "0 0 0"

        # inertia
        if base in dyn:
            mapping[f"{base}_mass"] = dyn[base].get("mass", 1.0)

            if "inertia_origin" in dyn[base]:
                xyz = dyn[base]["inertia_origin"]["xyz"]
                rpy = dyn[base]["inertia_origin"]["rpy"]
                mapping[f"{base}_inertial_xyz"] = " ".join(map(str, xyz))
                mapping[f"{base}_inertial_rpy"] = " ".join(map(str, rpy))
            else:
                mapping[f"{base}_inertial_xyz"] = "0 0 0"
                mapping[f"{base}_inertial_rpy"] = "0 0 0"

            if "inertia" in dyn[base]:
                inertia = dyn[base]["inertia"]
                mapping[f"{base}_ixx"] = inertia["ixx"]
                mapping[f"{base}_ixy"] = inertia["ixy"]
                mapping[f"{base}_ixz"] = inertia["ixz"]
                mapping[f"{base}_iyy"] = inertia["iyy"]
                mapping[f"{base}_iyz"] = inertia["iyz"]
                mapping[f"{base}_izz"] = inertia["izz"]
        else:
            mapping[f"{base}_mass"] = 1.0
            mapping[f"{base}_inertial_xyz"] = "0 0 0"
            mapping[f"{base}_inertial_rpy"] = "0 0 0"
            mapping[f"{base}_ixx"] = 0.001
            mapping[f"{base}_iyy"] = 0.001
            mapping[f"{base}_izz"] = 0.001
            mapping[f"{base}_ixy"] = mapping[f"{base}_ixz"] = mapping[f"{base}_iyz"] = 0.0

    # -----------------------------------------------------
    # Joint mapping
    # -----------------------------------------------------
    for j in kin:
        xyz = kin[j]["origin_xyz"]
        rpy = kin[j]["origin_rpy"]
        mapping[f"j_{j}_xyz"] = " ".join(map(str, xyz))
        mapping[f"j_{j}_rpy"] = " ".join(map(str, rpy))

        effort = ctl.get(j, {}).get("effort", 20.0)
        mapping[f"limit_{j}_effort"] = effort

    # -----------------------------------------------------
    # 渲染模板并保存
    # -----------------------------------------------------

    output_text = fill_template(template_text, mapping)
    outname = f"{robot_name}.xacro"
    save_text(outname, output_text)

    print(f"[OK] 生成: {outname}")
    print(f"[OK] 生成 joint limit 文件: {joint_limit_yaml}")


# =========================================================
# 程序入口
# =========================================================
if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("用法: python3 generate_spawn_xacro.py ur3_environment.json")
    else:
        generate_spawn(sys.argv[1])
