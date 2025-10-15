import os
import subprocess

URDF_DIR = "."  # 当前目录
CONFIG_BASE_DIR = "./config"
OUTPUT_BASE_DIR = "./output"

for filename in os.listdir(URDF_DIR):
    if filename.endswith(".urdf"):
        urdf_path = os.path.join(URDF_DIR, filename)
        robot_name = filename.replace(".urdf", "")
        config_dir = os.path.join(CONFIG_BASE_DIR, robot_name)
        output_dir = os.path.join(OUTPUT_BASE_DIR, robot_name)

        # 检查配置文件夹是否存在
        if not os.path.isdir(config_dir):
            print(f"[跳过] 缺少配置目录: {config_dir}")
            continue

        print(f"[生成] {robot_name}")
        subprocess.run([
            "python", "generate_aas_submodels_light.py",
            urdf_path, config_dir, output_dir
        ])
