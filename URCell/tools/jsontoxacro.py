import json
import os
import xml.etree.ElementTree as ET


def indent(elem, level=0):
    """为 XML 元素树添加缩进和换行，使输出更美观"""
    i = "\n" + level * "  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        for subelem in elem:
            indent(subelem, level + 1)
        if not subelem.tail or not subelem.tail.strip():
            subelem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i


def generate_minimal_xacro(json_data, output_path):
    """根据 AAS JSON 生成最小化 XACRO 文件"""
    robot = ET.Element("robot", {"name": "UR5"})

    for submodel in json_data['submodels']:
        for element in submodel['submodelElements']:
            if 'value' not in element:
                continue

            # link
            if element['idShort'].startswith('Link'):
                ET.SubElement(robot, "link", {"name": element['idShort']})

            # joint
            elif element['idShort'].startswith('Joint'):
                joint_name = element['idShort']
                parent = ""
                child = ""
                origin = None

                for prop in element['value']:
                    if prop['idShort'] == 'parent':
                        parent = prop['value']
                    elif prop['idShort'] == 'child':
                        child = prop['value']
                    elif prop['idShort'] == 'origin':
                        origin = prop['value']

                # 忽略 parent/child 为空的 joint
                if not parent or not child:
                    continue

                joint_elem = ET.SubElement(robot, "joint", {
                    "name": joint_name,
                    "type": "revolute"
                })
                ET.SubElement(joint_elem, "parent", {"link": parent})
                ET.SubElement(joint_elem, "child", {"link": child})

                # origin（仅保留前三个值）
                if origin:
                    try:
                        origin_data = eval(origin)
                        xyz = " ".join(map(str, origin_data["xyz"][:3]))
                        rpy = " ".join(map(str, origin_data["rpy"][:3]))
                        ET.SubElement(joint_elem, "origin", {"xyz": xyz, "rpy": rpy})
                    except Exception as e:
                        print(f"⚠️ origin 数据解析失败: {origin}, 错误: {e}")

                ET.SubElement(joint_elem, "prefit", {"value": "UR5"})

    # 添加缩进和换行
    indent(robot)

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    tree = ET.ElementTree(robot)
    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    print(f"✅ XACRO file generated at: {output_path}")


def main():
    prefix = r"C:\Users\benhe\Desktop\abschlussarbeit\URCell"
    input_json_path = os.path.join(prefix, "types", "submodel", "ur5", "ur5_environment.json")
    output_xacro_path = os.path.join(prefix, "projects", "cellA", "ur5_spawn.xacro")

    if not os.path.exists(input_json_path):
        print(f"❌ 输入文件不存在: {input_json_path}")
        return

    with open(input_json_path, "r", encoding="utf-8") as f:
        json_data = json.load(f)

    generate_minimal_xacro(json_data, output_xacro_path)


if __name__ == "__main__":
    main()
