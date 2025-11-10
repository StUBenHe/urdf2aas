import os
import json
from lxml import etree
from urdfpy import URDF
import numpy as np
from generate_aas_submodels_ur import (
    load_yaml,
    save_submodel,
    generate_structure_submodel,   # ✅ 使用双参数版本 (robot_name, xml_root)
    generate_control_submodel,
    generate_kinematics_submodel,
    generate_dynamics_submodel,
    generate_safety_submodel,
    generate_visualization_submodel,
    assemble_environment_v3,
)

# === ✅ limit 兼容性修复 ===
def patch_limit_compatibility(robot):
    """
    兼容性修复：
    - urdfpy 不同版本里 joint.limit 可能是 None / dict / 对象，且字段可能是 str / list / ndarray
    - 为 fixed/floating/planar 跳过
    - 为 continuous 关节提供默认 [-2π, 2π]
    - 统一输出为可读可写对象（有 .lower/.upper/.effort/.velocity）
    """
    import numpy as np
    TWO_PI = 2.0 * np.pi

    def _to_float(val, default=None):
        try:
            if val is None:
                return default
            # 取数组/列表的第一个元素
            if isinstance(val, (list, tuple, np.ndarray)):
                if len(val) == 0:
                    return default
                val = val[0]
            # 字符串转 float
            if isinstance(val, str):
                val = val.strip()
                if val == "":
                    return default
            return float(val)
        except Exception:
            return default

    for joint in getattr(robot, "joints", []):
        jt = getattr(joint, "joint_type", "fixed")
        if jt in ("fixed", "floating", "planar"):
            # 这些没有 limit 的需求，直接跳过
            continue

        lim = getattr(joint, "limit", None)

        # 先把现有 limit 抽取成原始数值
        if isinstance(lim, dict):
            lower   = _to_float(lim.get("lower"),   None)
            upper   = _to_float(lim.get("upper"),   None)
            effort  = _to_float(lim.get("effort"),  None)
            velocity= _to_float(lim.get("velocity"),None)
        else:
            lower   = _to_float(getattr(lim, "lower",   None), None)
            upper   = _to_float(getattr(lim, "upper",   None), None)
            effort  = _to_float(getattr(lim, "effort",  None), None)
            velocity= _to_float(getattr(lim, "velocity",None), None)

        # continuous 关节：URDF 常缺上下界，这里给个常见默认 [-2π, 2π]
        if jt == "continuous":
            if lower is None: lower = -TWO_PI
            if upper is None: upper =  TWO_PI
        else:
            # 其它关节的兜底
            if lower is None: lower = -np.pi
            if upper is None: upper =  np.pi

        if effort   is None: effort   = 0.0
        if velocity is None: velocity = 1.0

        # 确保 joint.limit 可写：如果没有对象或对象不可写，就造一个简单容器
        if lim is None or not hasattr(lim, "__dict__"):
            class _Limit: pass
            lim_obj = _Limit()
            setattr(joint, "limit", lim_obj)
        else:
            lim_obj = lim

        # 统一写回标准属性
        setattr(lim_obj, "lower",   float(lower))
        setattr(lim_obj, "upper",   float(upper))
        setattr(lim_obj, "effort",  float(effort))
        setattr(lim_obj, "velocity",float(velocity))


# === ✅ 运动学子模型 fallback 自动生成 ===
def generate_kinematics_submodel_fallback(kinematics_yaml, robot=None):
    import numpy as np

    def _safe_list(v):
        """安全地将 numpy 数组、元组、列表都转换为纯 Python list"""
        if v is None:
            return [0, 0, 0]
        if isinstance(v, np.ndarray):
            return v.tolist()
        if isinstance(v, (list, tuple)):
            return list(v)
        return [float(v), 0, 0] if isinstance(v, (int, float)) else [0, 0, 0]

    elements = []
    if not kinematics_yaml and robot is not None:
        print("⚠️ YAML缺失，自动生成基础运动学参数。")
        for joint in robot.joints:
            if joint.joint_type in ["revolute", "continuous", "prismatic"]:
                # --- origin ---
                xyz, rpy = [0, 0, 0], [0, 0, 0]
                if hasattr(joint, "origin") and joint.origin is not None:
                    try:
                        if hasattr(joint.origin, "xyz"):
                            xyz = _safe_list(joint.origin.xyz)
                        elif hasattr(joint.origin, "translation"):
                            xyz = _safe_list(joint.origin.translation)
                        if hasattr(joint.origin, "rpy"):
                            rpy = _safe_list(joint.origin.rpy)
                    except Exception:
                        xyz, rpy = [0, 0, 0], [0, 0, 0]

                # --- axis ---
                axis_val = getattr(joint, "axis", [0, 0, 1])
                axis_val = _safe_list(axis_val)

                elements.append({
                    "idShort": f"Kinematics_{joint.name}",
                    "modelType": "SubmodelElementCollection",
                    "value": {
                        "axis": axis_val,
                        "type": joint.joint_type,
                        "origin": {"xyz": xyz, "rpy": rpy}
                    }
                })
    else:
        # 有 YAML 时直接用 YAML 数据
        for joint_name, params in kinematics_yaml.items():
            elements.append({
                "idShort": f"Kinematics_{joint_name}",
                "modelType": "SubmodelElementCollection",
                "value": params
            })
    return {"idShort": "KinematicsSubmodel", "submodelElements": elements}

def main_igus_rebel():
    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    igus_root = os.path.join(base_dir, "types", "igus_rebel_description_ros2")
    submodel_root = os.path.join(base_dir, "types", "submodel", "igus_rebel")

    urdf_path = os.path.join(igus_root, "urdf", "igus_rebel.urdf")
    yaml_dir = os.path.join(igus_root, "config", "igus_rebel_6dof")
    output_dir = submodel_root

    if not os.path.exists(urdf_path):
        print(f"❌ 未找到 URDF 文件: {urdf_path}")
        return

    os.makedirs(yaml_dir, exist_ok=True)
    os.makedirs(output_dir, exist_ok=True)

    print(f"🤖 正在加载 IGUS Rebel URDF: {urdf_path}")

    with open(urdf_path, "r", encoding="utf-8") as f:
        urdf_text = f.read().replace("\\", "/")

    # ✅ 替换正确的包路径前缀
    package_root = os.path.join(base_dir, "types", "igus_rebel_description_ros2").replace("\\", "/")
    urdf_text = urdf_text.replace("package://igus_rebel_description_ros2/", package_root + "/")

    parser = etree.XMLParser(remove_blank_text=True)
    root = etree.fromstring(urdf_text.encode("utf-8"), parser=parser)

    import trimesh
    _original_trimesh_load = trimesh.load
    trimesh.load = lambda *a, **kw: trimesh.Trimesh()  # 防止 STL/DAE 加载失败

    try:
        robot = URDF._from_xml(root, None)
    finally:
        trimesh.load = _original_trimesh_load

    patch_limit_compatibility(robot)
    print(f"✅ 成功加载模型: {robot.name}, 含 {len(robot.links)} 个 link, {len(robot.joints)} 个 joint。")

    # === 载入配置 ===
    joint_limits = load_yaml(os.path.join(yaml_dir, "joint_limits.yaml"))
    kinematics = load_yaml(os.path.join(yaml_dir, "default_kinematics.yaml"))
    physical = load_yaml(os.path.join(yaml_dir, "physical_parameters.yaml"))
    visual = load_yaml(os.path.join(yaml_dir, "visual_parameters.yaml"))
    base_name = "igus_rebel_6dof"
    control_template_path = os.path.join(yaml_dir, f"{base_name}_control.json")

    # === ⚙️ 子模型生成 ===
    # StructureSubmodel 从 XML 提取（支持 axis / dynamics / 材质）
    save_submodel(generate_structure_submodel(base_name, root), f"{base_name}_structure_submodel", output_dir)

    # Control Submodel - 增加 transmission 兜底
    save_submodel(generate_control_submodel(robot, control_template_path, joint_limits), f"{base_name}_control_submodel", output_dir)

    # Kinematics Submodel - 支持 fallback
    save_submodel(generate_kinematics_submodel_fallback(kinematics, robot), f"{base_name}_kinematics_submodel", output_dir)

    # Dynamics Submodel - 保持物理参数一致
    save_submodel(generate_dynamics_submodel(robot, physical), f"{base_name}_dynamics_submodel", output_dir)

    # Safety Submodel
    save_submodel(generate_safety_submodel(robot), f"{base_name}_safety_submodel", output_dir)

    # Visualization Submodel
    save_submodel(generate_visualization_submodel(robot, visual), f"{base_name}_visualization_submodel", output_dir)

    assemble_environment_v3(base_name, output_dir)
    print(f"🎯 IGUS Rebel 全部 AAS 子模型生成完成，已保存至: {output_dir}")


if __name__ == "__main__":
    main_igus_rebel()
