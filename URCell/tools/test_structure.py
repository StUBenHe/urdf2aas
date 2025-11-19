from structure_generator import generate_xacro_structure

generate_xacro_structure(
    env_json_path="../types/submodel/ur5/ur5_environment.json",
    robot_type="ur5",
    output_path="../projects/spawns/ur5_spawn.xacro"
)
