import io
import os
import sys
import builtins
import xacro
from xml.etree import ElementTree as ET

# === 路径配置 ===
base_path = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
input_file = os.path.join(base_path, "projects", "multi_ur.xacro")
output_file = os.path.join(base_path, "projects", "multi_ur.urdf")

print("🔧 运行 Xacro 展开：")
print(f"  输入文件: {input_file}")
print(f"  输出文件: {output_file}")

# === 捕获文件打开以确定哪个文件触发 GBK ===
_builtin_open = builtins.open

def tracing_open(file, mode='r', *args, **kwargs):
    # 对每个打开的文件打印一次路径
    if isinstance(file, str):
        print(f"📂 打开文件: {file}")
    try:
        # 所有文本模式都强制 UTF-8
        if 'b' not in mode:
            kwargs['encoding'] = 'utf-8'
        return _builtin_open(file, mode, *args, **kwargs)
    except Exception as e:
        print(f"⚠️ 打开文件失败: {file} ({e})")
        raise

# ✅ 替换全局 open
builtins.open = tracing_open

try:
    with io.open(input_file, "r", encoding="utf-8") as f:
        xml_text = f.read()

    # 解析 & 展开
    doc = xacro.parse(io.StringIO(xml_text))
    xacro.process_doc(doc)

    ET.ElementTree(doc.documentElement).write(
        output_file, encoding="utf-8", xml_declaration=True
    )

    print(f"\n✅ 已生成 URDF 文件：{output_file}")

except Exception as e:
    print(f"\n❌ 生成失败：{e}")
