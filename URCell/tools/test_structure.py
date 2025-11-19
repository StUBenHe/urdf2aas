from structure_generator import generate_xacro_structure

generate_xacro_structure(
    env_json_path="../types/submodel/ur3/ur3_environment.json",
    robot_type="ur3",
    output_path="../projects/spawns/ur3_spawn.xacro"
)
