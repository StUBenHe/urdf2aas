import os

root = r"C:\Users\benhe\Desktop\abschlussarbeit\URCell"

def convert_to_utf8(file_path):
    try:
        with open(file_path, "r", encoding="gbk") as f:
            text = f.read()
        with open(file_path, "w", encoding="utf-8") as f:
            f.write(text)
        print(f"✅ 转换成功: {file_path}")
    except UnicodeDecodeError:
        # 尝试 UTF-8 读取，如果本身就是 UTF-8 就跳过
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                f.read()
            print(f"✅ 已是 UTF-8: {file_path}")
        except Exception as e:
            print(f"⚠️ 无法识别编码: {file_path} ({e})")

for dirpath, _, filenames in os.walk(root):
    for filename in filenames:
        if filename.endswith(".xacro"):
            convert_to_utf8(os.path.join(dirpath, filename))

print("\n🎯 所有 .xacro 文件已转换为 UTF-8 编码！")
