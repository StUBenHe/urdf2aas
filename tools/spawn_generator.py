import os
import json


def extract_mesh_path(mesh_path: str) -> str:
    """
    Clean the mesh path from AAS definitions.

    Examples:
        Input:
            package://ur_description/meshes/ur3/visual/base.dae
            package:://igus_rebel_description_ros2/meshes/igus_rebel/visual/base.dae

        Output:
            ur_description/meshes/ur3/visual/base.dae
            igus_rebel_description_ros2/meshes/igus_rebel/visual/base.dae
    """
    if not mesh_path or not isinstance(mesh_path, str):
        return

    # The igus model sometimes contains the typo "package:://", so normalize it
    mesh_path = mesh_path.replace("package:://", "")
    mesh_path = mesh_path.replace("package://", "")

    # Remove leading slash if still present
    mesh_path = mesh_path.lstrip("/")

    return mesh_path


def parse_json_str(value):
    """
    Convert a JSON string into a Python object.
    If already a dict/list, return as is.
    If conversion fails, return None.
    """
    if isinstance(value, (dict, list)):
        return value
    if isinstance(value, str):
        try:
            return json.loads(value)
        except Exception:
            return None
    return None


def make_origin(origin_json):
    """
    Convert an origin JSON into a URDF <origin> tag string.

    origin_json format example:
        {"xyz": ["0","0","0"], "rpy": ["0","0","0"]}

    If origin_json is missing, return identity origin.
    """
    if not origin_json:
        return '<origin xyz="0 0 0" rpy="0 0 0"/>'
    xyz = origin_json.get("xyz", ["0", "0", "0"])
    rpy = origin_json.get("rpy", ["0", "0", "0"])
    return f'<origin xyz="{xyz[0]} {xyz[1]} {xyz[2]}" rpy="{rpy[0]} {rpy[1]} {rpy[2]}"/>'


def extract_translation_from_matrix(mat):
    """
    Extract translation (xyz) from a 4x4 matrix.
    Rotation is ignored; rpy defaults to zero.
    """
    try:
        tx = float(mat[0][3])
        ty = float(mat[1][3])
        tz = float(mat[2][3])
    except:
        tx = ty = tz = 0.0

    return {"xyz": [str(tx), str(ty), str(tz)], "rpy": ["0", "0", "0"]}


def generate_xacro_structure(env_json_path, robot_type, output_dir):
    """
    Generate a spawn.xacro structure from an AAS environment.json.

    This produces:
        output_dir/<robot_type>_spawn.xacro

    It constructs:
        - Links
        - Visual/collision meshes
        - Inertial sections
        - Joints
        - xacro macro wrapper
    """
    with open(env_json_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    prefix = "${prefix}"
    mesh_root = "${mesh_root}"
    robot_type_lower = robot_type.lower()

    lines = []

    # Locate Structure and Dynamics submodels
    structure_sm = None
    dynamics_sm = None

    for sm in data["submodels"]:
        sid = sm.get("idShort", "")
        if "StructureSubmodel" in sid:
            structure_sm = sm
        elif "DynamicsSubmodel" in sid:
            dynamics_sm = sm

    if structure_sm is None:
        raise RuntimeError("StructureSubmodel missing in environment.json")

    struct_elems = {e["idShort"]: e for e in structure_sm["submodelElements"]}
    dyn_elems = {e["idShort"]: e for e in (dynamics_sm["submodelElements"] if dynamics_sm else [])}

    # Hardcoded UR5 link and joint order
    LINK_ORDER = [
        "Link_base_link",
        "Link_base_link_inertia",
        "Link_shoulder_link",
        "Link_upper_arm_link",
        "Link_forearm_link",
        "Link_wrist_1_link",
        "Link_wrist_2_link",
        "Link_wrist_3_link"
    ]

    JOINT_ORDER = [
        "Joint_base_link-base_link_inertia",
        "Joint_shoulder_pan_joint",
        "Joint_shoulder_lift_joint",
        "Joint_elbow_joint",
        "Joint_wrist_1_joint",
        "Joint_wrist_2_joint",
        "Joint_wrist_3_joint"
    ]

    # ---------------------------------------------------
    # XML HEADER
    # ---------------------------------------------------
    lines.append('<?xml version="1.0"?>')
    lines.append('<robot xmlns:xacro="http://www.ros.org/wiki/xacro">')
    lines.append('')
    lines.append(f'<xacro:macro name="{robot_type_lower}_spawn" params="prefix xyz rpy">')
    lines.append('  <xacro:property name="mesh_root" value="/home/unvrx/URCell/types/" />')
    lines.append(
        f'  <xacro:property name="joint_limits" value="${{xacro.load_yaml(\'spawns/{robot_type_lower}_joint_limits.yaml\')}}"/>'
    )

    # ---------------------------------------------------
    # LINKS
    # ---------------------------------------------------
    lines.append('  <!-- LINKS -->')

    for key in LINK_ORDER:
        link_name = key.replace("Link_", "")
        struct_elem = struct_elems.get(key)
        dyn_elem = dyn_elems.get(f"Dynamics_{link_name}")

        tag = f"{prefix}{link_name}"

        # Special case: base_link is empty
        if link_name == "base_link":
            lines.append(f'  <link name="{tag}" />\n')
            continue

        # Special case: base_link_inertia
        if link_name == "base_link_inertia":
            lines.append(f'  <link name="{tag}">')

            visual_mesh = None
            collision_mesh = None

            if struct_elem:
                for group in struct_elem.get("value", []):
                    if group.get("idShort") == "visuals":
                        for vis in group.get("value", []):
                            props = {p["idShort"]: p.get("value") for p in vis.get("value", [])}
                            if props.get("mesh"):
                                visual_mesh = props["mesh"]
                                break
                    if group.get("idShort") == "collisions":
                        for col in group.get("value", []):
                            props = {p["idShort"]: p.get("value") for p in col.get("value", [])}
                            if props.get("mesh"):
                                collision_mesh = props["mesh"]
                                break

            # Visual block
            lines.append('    <visual>')
            lines.append('      <origin xyz="0 0 0" rpy="0 0 3.141592653589793"/>')
            if visual_mesh:
                rel_path = extract_mesh_path(visual_mesh)
                lines.append(
                    f'      <geometry><mesh filename="file://${{mesh_root}}{rel_path}"/></geometry>'
                )
            lines.append('      <material name="LightGrey"><color rgba="0.7 0.7 0.7 1.0"/></material>')
            lines.append('    </visual>')

            # Collision block
            lines.append('    <collision>')
            lines.append('      <origin xyz="0 0 0" rpy="0 0 3.141592653589793"/>')
            if collision_mesh:
                rel_path = extract_mesh_path(collision_mesh)
                lines.append(
                    f'      <geometry><mesh filename="file://${{mesh_root}}{rel_path}"/></geometry>'
                )
            lines.append('    </collision>')

            # Inertial from AAS
            if dyn_elem:
                props = {v["idShort"]: v.get("value") for v in dyn_elem["value"] if v["modelType"] == "Property"}
                mass = props.get("mass", "1.0")
                inertia = parse_json_str(props.get("inertia"))
            else:
                mass = "1.0"
                inertia = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

            lines.append('    <inertial>')
            lines.append(f'      <mass value="{mass}"/>')
            lines.append(
                f'      <inertia ixx="{inertia[0][0]}" ixy="{inertia[0][1]}" ixz="{inertia[0][2]}" '
                f'iyy="{inertia[1][1]}" iyz="{inertia[1][2]}" izz="{inertia[2][2]}"/>'
            )
            lines.append('      <origin xyz="0 0 0" rpy="0 0 0"/>')
            lines.append('    </inertial>')
            lines.append('  </link>\n')
            continue

        # Normal link definition
        lines.append(f'  <link name="{tag}">')

        # Inertial block
        mass = None
        inertia = None
        inertia_origin = None

        if struct_elem:
            sprops = {
                v["idShort"]: v.get("value")
                for v in struct_elem.get("value", [])
                if v.get("modelType") == "Property"
            }
            mass = sprops.get("mass")
            inertia = parse_json_str(sprops.get("inertia"))

        if dyn_elem:
            dprops = {
                v["idShort"]: v.get("value")
                for v in dyn_elem.get("value", [])
                if v.get("modelType") == "Property"
            }

            if mass is None:
                mass = dprops.get("mass")
            if inertia is None:
                inertia = parse_json_str(dprops.get("inertia"))

            mat = parse_json_str(dprops.get("origin"))
            if isinstance(mat, list):
                inertia_origin = extract_translation_from_matrix(mat)

        if mass and inertia:
            lines.append('    <inertial>')
            lines.append(f'      <mass value="{mass}"/>')
            lines.append(
                f'      <inertia ixx="{inertia[0][0]}" ixy="{inertia[0][1]}" ixz="{inertia[0][2]}" '
                f'iyy="{inertia[1][1]}" iyz="{inertia[1][2]}" izz="{inertia[2][2]}"/>'
            )
            if inertia_origin:
                lines.append(f'      {make_origin(inertia_origin)}')
            lines.append('    </inertial>')

        # Visual
        if struct_elem:
            for group in struct_elem.get("value", []):
                if group.get("idShort") == "visuals":
                    for vis in group.get("value", []):
                        props = {p["idShort"]: p.get("value") for p in vis.get("value", [])}
                        mesh = props.get("mesh")
                        origin = parse_json_str(props.get("origin"))
                        if mesh:
                            rel_path = extract_mesh_path(mesh)
                            lines.append('    <visual>')
                            lines.append(make_origin(origin))
                            lines.append(
                                f'      <geometry><mesh filename="file://${{mesh_root}}{rel_path}"/></geometry>'
                            )
                            lines.append(
                                '      <material name="LightGrey"><color rgba="0.7 0.7 0.7 1.0"/></material>'
                            )
                            lines.append('    </visual>')

        # Collision
        if struct_elem:
            for group in struct_elem.get("value", []):
                if group.get("idShort") == "collisions":
                    for col in group.get("value", []):
                        props = {p["idShort"]: p.get("value") for p in col.get("value", [])}
                        mesh = props.get("mesh")
                        origin = parse_json_str(props.get("origin"))
                        if mesh:
                            rel_path = extract_mesh_path(mesh)
                            lines.append('    <collision>')
                            lines.append(make_origin(origin))
                            lines.append(
                                f'      <geometry><mesh filename="file://${{mesh_root}}{rel_path}"/></geometry>'
                            )
                            lines.append('    </collision>')

        lines.append("  </link>\n")

    # ---------------------------------------------------
    # JOINTS
    # ---------------------------------------------------
    lines.append("  <!-- JOINTS -->")

    for key in JOINT_ORDER:
        elem = struct_elems.get(key)
        if not elem:
            continue

        jname = key.replace("Joint_", "")
        tag = f"{prefix}{jname}"

        props = {v["idShort"]: v.get("value") for v in elem["value"]}

        jtype = props.get("type", "fixed")
        parent = f'{prefix}{props.get("parent")}'
        child = f'{prefix}{props.get("child")}'
        origin = parse_json_str(props.get("origin"))

        lines.append(f'  <joint name="{tag}" type="{jtype}">')
        lines.append(f'    <parent link="{parent}"/>')
        lines.append(f'    <child link="{child}"/>')
        lines.append(f'    {make_origin(origin)}')

        if jtype == "revolute":
            lines.append('    <axis xyz="0 0 1"/>')
            lines.append(
                f'    <limit effort="150.0"\n'
                f'      lower="${{joint_limits.joint_limits.{jname}.min_position}}"\n'
                f'      upper="${{joint_limits.joint_limits.{jname}.max_position}}"\n'
                f'      velocity="${{joint_limits.joint_limits.{jname}.max_velocity}}"/>'
            )

        lines.append("  </joint>\n")

    # ---------------------------------------------------
    # FOOTER
    # ---------------------------------------------------
    lines.append("</xacro:macro>")
    lines.append("</robot>")

    output_file = os.path.join(output_dir, f"{robot_type}_spawn.xacro")

    os.makedirs(os.path.dirname(output_file), exist_ok=True)

    with open(output_file, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))

    print("Generation completed:", output_file)
