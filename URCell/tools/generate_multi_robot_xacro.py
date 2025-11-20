import os
import yaml

from joint_limit_generator import generate_joint_limits

from spawn_generator import generate_xacro_structure


def generate_robot_block(robot):
    """
    根据 robot 字段生成 joint + xacro 实例
    """
    name = robot["name"]
    rtype = robot["type"]
    xyz = robot["xyz"]
    rpy = robot["rpy"]

    xyz_str = " ".join(str(v) for v in xyz)
    rpy_str = " ".join(str(v) for v in rpy)

    spawn_macro = f"{rtype}_spawn"  # 必须与 spawns/ 中的文件一致

    xml = f"""
  <joint name="world_to_{name}" type="fixed">
    <parent link="world"/>
    <child link="{name}_base_link"/>
    <origin xyz="{xyz_str}" rpy="{rpy_str}"/>
  </joint>

  <xacro:{spawn_macro} prefix="{name}_" xyz="{xyz_str}" rpy="{rpy_str}"/>
"""
    return xml


def generate_xacro(multi_yaml, output_file="multi_ur.xacro"):
    """
    主入口：
     - 自动生成 joint_limits
     - include spawn xacro
     - 生成 multi_ur.xacro
    """
    with open(multi_yaml, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    robots = data.get("robots", [])

    base_dir = os.path.dirname(os.path.abspath(multi_yaml))
    spawn_dir = os.path.join(base_dir, "spawns")
    os.makedirs(spawn_dir, exist_ok=True)

    submodel_base = os.path.abspath(
        os.path.join(base_dir, "../types/submodel")
    )
    # 收集所有类型 → include spawn xacro
    spawn_includes = set()

    def is_ur_series(rtype: str) -> bool:
        """
        判断机器人是否为 UR 系列
        """
        r = rtype.lower()
        return r.startswith("ur")  # ur3, ur5e, ur10e, ur20… 均匹配

    for robot in robots:
        rtype = robot["type"]

        # ⭐ 1. 所有 robot 都加入 spawn_includes（你需要的）
        spawn_includes.add(rtype)

        # ⭐ 2. 非 UR 系列：跳过生成
        if not is_ur_series(rtype):
            print(f"⏭ 非 UR 机器人，不生成 joint limits 和 xacro：{rtype}")
            continue

        # ⭐ 3. UR 系列：生成 joint limits + xacro
        env_json_path = os.path.join(
            submodel_base, rtype, f"{rtype}_environment.json"
        )

        if not os.path.exists(env_json_path):
            print(f"❌ 找不到 environment.json: {env_json_path}")
            continue

        print(f"📄 发现 environment.json: {env_json_path}")

        generate_joint_limits(
            env_json_path=env_json_path,
            robot_type=rtype,
            output_dir=spawn_dir
        )

        generate_xacro_structure(
            env_json_path=env_json_path,
            robot_type=rtype,
            output_dir=spawn_dir
        )

    # ---- 生成 xacro 文件 ----

    xml_output = """<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="multi_ur">

"""

    # include 所有 spawn.xacro
    for t in spawn_includes:
        xml_output += f'  <xacro:include filename="spawns/{t}_spawn.xacro"/>\n'

    xml_output += """
  <link name="world"/>
"""

    # 每个机器人 block
    for robot in robots:
        xml_output += generate_robot_block(robot)

    xml_output += "\n</robot>\n"

    # 写入 multi_ur.xacro
    with open(output_file, "w", encoding="utf-8") as f:
        f.write(xml_output)

    print(f"🎉 已生成: {output_file}")




if __name__ == "__main__":
    # 默认路径（你可修改）
    generate_xacro(
        "../projects/multi_ur.yaml",
        "../projects/multi_ur.xacro"
    )
