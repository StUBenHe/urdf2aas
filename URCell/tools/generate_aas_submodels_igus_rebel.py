import os
import json
from lxml import etree
from urdfpy import URDF
import numpy as np
from generate_aas_submodels_ur import (
    load_yaml,
    save_submodel,
    generate_structure_submodel,
    generate_control_submodel,
    generate_kinematics_submodel,
    generate_dynamics_submodel,
    generate_safety_submodel,
    generate_visualization_submodel,
    assemble_environment_v3,
)

# === ✅ limit 兼容性修复 ===
def patch_limit_compatibility(robot):
    for joint in robot.joints:
        if not hasattr(joint, "limit") or joint.limit is None:
            continue
        limit = joint.limit
        lower = getattr(limit, "lower", None)
        upper = getattr(limit, "upper", None)
        effort = getattr(limit, "effort", None)
        velocity = getattr(limit, "velocity", None)
        if isinstance(limit, dict):
            lower = limit.get("lower", lower)
            upper = limit.get("upper", upper)
            effort = limit.get("effort", effort)
            velocity = limit.get("velocity", velocity)
        joint.limit.lower = float(lower if lower is not None else -3.14)
        joint.limit.upper = float(upper if upper is not None else 3.14)
        joint.limit.effort = float(effort if effort is not None else 0.0)
        joint.limit.velocity = float(velocity if velocity is not None else 1.0)


def main_igus_rebel():
    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    igus_root = os.path.join(base_dir, "types", "igus_rebel_description_ros2")
    submodel_root = os.path.join(base_dir, "types", "submodel", "igus_rebel")

    urdf_path = os.path.join(igus_root, "urdf", "igus_rebel.urdf")
    yaml_dir = os.path.join(igus_root, "config", "igus_rebel_6dof")
    output_dir = os.path.join(submodel_root, "igus_rebel_6dof")

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

    # ✅ 全局禁用 trimesh 加载，防止 .dae/.stl 触发错误
    _original_trimesh_load = trimesh.load
    trimesh.load = lambda *a, **kw: trimesh.Trimesh()  # 返回一个空 mesh

    try:
        parser = etree.XMLParser(remove_blank_text=True)
        root = etree.fromstring(urdf_text.encode("utf-8"), parser=parser)
        robot = URDF._from_xml(root, None)
    finally:
        # 恢复原始函数，避免影响其他模块
        trimesh.load = _original_trimesh_load

    # ✅ 修正 limit 兼容性
    patch_limit_compatibility(robot)

    print(f"✅ 成功加载模型: {robot.name}, 含 {len(robot.links)} 个 link, {len(robot.joints)} 个 joint。")

    joint_limits = load_yaml(os.path.join(yaml_dir, "joint_limits.yaml"))
    kinematics = load_yaml(os.path.join(yaml_dir, "default_kinematics.yaml"))
    physical = load_yaml(os.path.join(yaml_dir, "physical_parameters.yaml"))
    visual = load_yaml(os.path.join(yaml_dir, "visual_parameters.yaml"))
    base_name = "igus_rebel_6dof"
    control_template_path = os.path.join(yaml_dir, f"{base_name}_control.json")

    save_submodel(generate_structure_submodel(robot), f"{base_name}_structure_submodel", output_dir)
    save_submodel(generate_control_submodel(robot, control_template_path, joint_limits), f"{base_name}_control_submodel", output_dir)
    save_submodel(generate_kinematics_submodel(kinematics), f"{base_name}_kinematics_submodel", output_dir)
    save_submodel(generate_dynamics_submodel(robot, physical), f"{base_name}_dynamics_submodel", output_dir)
    save_submodel(generate_safety_submodel(robot), f"{base_name}_safety_submodel", output_dir)
    save_submodel(generate_visualization_submodel(robot, visual), f"{base_name}_visualization_submodel", output_dir)

    assemble_environment_v3(base_name, output_dir)
    print(f"🎯 IGUS Rebel 全部 AAS 子模型生成完成，已保存至: {output_dir}")


if __name__ == "__main__":
    main_igus_rebel()
