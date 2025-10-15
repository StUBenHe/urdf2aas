from pathlib import Path

# URDF 文件路径
urdf_path = Path("C:/Users/benhe/Desktop/abschlussarbeit/urdf/pythonProject/ur3.urdf")

# 实际 mesh 路径（注意！你之前说 mesh 放在 pythonProject/meshes）
mesh_base_path = Path("C:/Users/benhe/Desktop/abschlussarbeit/urdf/pythonProject/meshes")

# 读取并替换
text = urdf_path.read_text(encoding='utf-8')
new_text = text.replace("package://ur_description/", str(mesh_base_path.as_posix()) + "/")

# 备份旧文件（如果已存在则删除）
backup = urdf_path.with_suffix(".urdf.bak")
if backup.exists():
    backup.unlink()
urdf_path.rename(backup)

# 写入替换后的 URDF
urdf_path.write_text(new_text, encoding='utf-8')
print("✅ 替换完成，路径指向实际 meshes 文件夹")
