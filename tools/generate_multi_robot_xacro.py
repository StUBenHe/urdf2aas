import os
import yaml

from joint_limit_generator import generate_joint_limits
from spawn_generator import generate_xacro_structure


def generate_robot_block(robot):
    """
    Generate a joint + xacro instance block for a robot entry.

    Parameters:
        robot (dict): A dictionary containing fields:
            - name: robot instance name
            - type: robot type (e.g., ur5, ur10e)
            - xyz: position in world frame
            - rpy: orientation (roll, pitch, yaw)

    Returns:
        str: XML string containing the joint and xacro macro call.
    """
    name = robot["name"]
    rtype = robot["type"]
    xyz = robot["xyz"]
    rpy = robot["rpy"]

    xyz_str = " ".join(str(v) for v in xyz)
    rpy_str = " ".join(str(v) for v in rpy)

    spawn_macro = f"{rtype}_spawn"  # must match files in the `spawns/` directory

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
    Main entry point:
      - Automatically generates joint limits
      - Includes spawn xacros
      - Builds the final multi_ur.xacro file

    Parameters:
        multi_yaml (str): Path to the YAML file describing multiple robots.
        output_file (str): Output xacro filename.
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

    # Collect all robot types → we will include their spawn xacro
    spawn_includes = set()

    def is_ur_series(rtype: str) -> bool:
        """
        Determine if a robot type belongs to the UR series.
        Matches: ur3, ur5e, ur10, ur20, ur30, etc.

        Parameters:
            rtype (str): Robot type name.

        Returns:
            bool: True if UR series, False otherwise.
        """
        r = rtype.lower()
        return r.startswith("ur")

    for robot in robots:
        rtype = robot["type"]

        # 1. Add all robot types to spawn includes
        spawn_includes.add(rtype)

        # 2. Skip non-UR series robots
        if not is_ur_series(rtype):
            print(f"⏭ Skipping non-UR robot: {rtype}")
            continue

        # 3. UR series → generate joint limits & xacro files
        env_json_path = os.path.join(
            submodel_base, rtype, f"{rtype}_environment.json"
        )

        if not os.path.exists(env_json_path):
            print(f" environment.json not found: {env_json_path}")
            continue

        print(f" Found environment.json: {env_json_path}")

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

    # ---- Build final xacro file ----

    xml_output = """<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="multi_ur">

"""

    # Include all spawn xacros
    for t in spawn_includes:
        xml_output += f'  <xacro:include filename="spawns/{t}_spawn.xacro"/>\n'

    xml_output += """
  <link name="world"/>
"""

    # Generate blocks for every robot
    for robot in robots:
        xml_output += generate_robot_block(robot)

    xml_output += "\n</robot>\n"

    # Write the output file
    with open(output_file, "w", encoding="utf-8") as f:
        f.write(xml_output)

    print(f" Generated: {output_file}")


if __name__ == "__main__":
    # Default paths (modify as needed)
    generate_xacro(
        "../projects/multi_ur.yaml",
        "../projects/multi_ur.xacro"
    )
