import json
from joint_limit_generator import load_joint_limits
from structure_generator import generate_xacro_structure

# ========== 测试 joint_limits ==========
print("=== 测试 joint_limits 生成 ===")

with open("../types/submodel/ur5/ur5_environment.json", "r", encoding="utf-8") as f:
    env_data = json.load(f)

limits = load_joint_limits(
    robot_type="ur5",
    env_data=env_data,
    spawn_dir="../projects/spawns"
)

print("joint_limits 已生成:\n", limits)


# ========== 测试 Structure Xacro 生成 ==========
print("\n=== 测试 structure xacro 生成 ===")

generate_xacro_structure(
    env_json_path="../types/submodel/ur5/ur5_environment.json",
    robot_type="ur5",
    output_path="../projects/spawns/ur5_structure.xacro"
)

print("结构 xacro 生成完毕！")
