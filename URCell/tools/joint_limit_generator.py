import os
import yaml
import json
import math

def generate_joint_limits(env_json_path, robot_type, output_dir):
    with open(env_json_path, "r", encoding="utf-8") as f:
        env_data = json.load(f)
    return load_joint_limits(robot_type, env_data, output_dir)


def rad_to_deg(rad):
    """弧度转角度"""
    return rad * 180.0 / math.pi


def clean_joint_name(jname):
    """
    去掉 AAS JSON 自动加的:
    Joint_XXX_Control → XXX
    Joint_XXX_Safety → XXX
    """
    if jname.startswith("Joint_"):
        jname = jname[len("Joint_"):]
    if jname.endswith("_Control"):
        jname = jname[:-len("_Control")]
    if jname.endswith("_Safety"):
        jname = jname[:-len("_Safety")]
    return jname


def load_joint_limits(robot_type, env_data, output_dir):
    """生成接近 UR 官方格式的 joint_limits.yaml"""

    out_file = os.path.join(output_dir, f"{robot_type}_joint_limits.yaml")

    joints_tmp = {}

    # 先从 Control Submodel 获取主数据
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

    # 补齐 missing fields（UR 官方格式）
    for j, vals in joints_tmp.items():
        vals.setdefault("has_acceleration_limits", False)
        vals.setdefault("has_effort_limits", "max_effort" in vals)
        vals.setdefault("has_position_limits", "min_position" in vals and "max_position" in vals)
        vals.setdefault("has_velocity_limits", "max_velocity" in vals)

    # 输出 YAML
    final_yaml = {"joint_limits": {}}
    for j, vals in joints_tmp.items():
        # 使用 degree 标签输出
        formatted = {}
        for key, value in vals.items():
            if key in ["min_position", "max_position", "max_velocity"]:
                formatted[key] = f"!degrees {value}"
            else:
                formatted[key] = value

        final_yaml["joint_limits"][j] = formatted

    os.makedirs(output_dir, exist_ok=True)
    with open(out_file, "w", encoding="utf-8") as f:
        yaml.dump(final_yaml, f, sort_keys=False, allow_unicode=True)

    return final_yaml
