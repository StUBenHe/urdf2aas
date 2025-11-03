import os, json, xml.etree.ElementTree as ET

def create_property(id_short, value, prefix):
    return {
        "idShort": id_short,
        "modelType": "Property",
        "valueType": "string",
        "value": str(value),
        "semanticId": {
            "keys": [{"type": "GlobalReference", "value": f"{prefix}{id_short}"}]
        }
    }

def create_collection(id_short, elements, prefix):
    return {
        "idShort": id_short,
        "modelType": "SubmodelElementCollection",
        "semanticId": {
            "keys": [{"type": "GlobalReference", "value": f"{prefix}{id_short}"}]
        },
        "value": elements
    }

def generate_aas_submodels_rebel():
    urdf_path = r"C:\\Users\\benhe\\Desktop\\abschlussarbeit\\URCell\\types\\igus_rebel_description_ros2\\urdf\\igus_rebel.urdf"
    output_dir = r"C:\\Users\\benhe\\Desktop\\abschlussarbeit\\URCell\\types\\submodel\\igus_rebel"
    prefix = "https://igus.de/aas/igus_rebel/"

    os.makedirs(output_dir, exist_ok=True)
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    def make_submodel(idshort, elements):
        return {
            "idShort": idshort,
            "modelType": "Submodel",
            "semanticId": {"keys": [{"type": "GlobalReference", "value": f"{prefix}{idshort}"}]},
            "submodelElements": elements
        }

    # --- StructureSubmodel ---
    structure = []
    for link in root.findall("link"):
        elements = [create_property("type", "link", prefix)]
        structure.append(create_collection(link.attrib.get("name"), elements, prefix))
    for joint in root.findall("joint"):
        elements = [
            create_property("type", joint.attrib.get("type"), prefix),
            create_property("parent", joint.find("parent").attrib.get("link"), prefix),
            create_property("child", joint.find("child").attrib.get("link"), prefix)
        ]
        structure.append(create_collection(joint.attrib.get("name"), elements, prefix))

    # --- KinematicsSubmodel ---
    kinematics = []
    for joint in root.findall("joint"):
        origin = joint.find("origin")
        axis = joint.find("axis")
        elements = [
            create_property("type", joint.attrib.get("type"), prefix),
            create_property("origin_xyz", origin.attrib.get("xyz", "0 0 0") if origin is not None else "0 0 0", prefix),
            create_property("origin_rpy", origin.attrib.get("rpy", "0 0 0") if origin is not None else "0 0 0", prefix),
            create_property("axis", axis.attrib.get("xyz") if axis is not None else "None", prefix)
        ]
        kinematics.append(create_collection(joint.attrib.get("name"), elements, prefix))

    # --- DynamicsSubmodel ---
    dynamics = []
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is not None:
            mass = inertial.find("mass").attrib.get("value") if inertial.find("mass") is not None else "None"
            inertia = inertial.find("inertia")
            elements = [create_property("mass", mass, prefix)]
            if inertia is not None:
                for k, v in inertia.attrib.items():
                    elements.append(create_property(k, v, prefix))
            dynamics.append(create_collection(link.attrib.get("name"), elements, prefix))

    # --- ControlSubmodel ---
    control = []
    for t in root.findall("transmission"):
        elements = []
        if t.find("joint") is not None:
            elements.append(create_property("joint", t.find("joint").attrib.get("name"), prefix))
        if t.find("actuator") is not None:
            elements.append(create_property("actuator", t.find("actuator").attrib.get("name"), prefix))
        control.append(create_collection(t.attrib.get("name"), elements, prefix))

    # --- SafetySubmodel ---
    safety = []
    for j in root.findall("joint"):
        limit = j.find("limit")
        if limit is not None:
            elements = [create_property(k, v, prefix) for k, v in limit.attrib.items()]
            safety.append(create_collection(j.attrib.get("name"), elements, prefix))

    # --- EndEffectorSubmodel ---
    endeffector = []
    for l in root.findall("link"):
        if any(k in l.attrib.get("name", "").lower() for k in ["ee", "flange", "tool"]):
            elements = [create_property("type", "endeffector", prefix)]
            endeffector.append(create_collection(l.attrib.get("name"), elements, prefix))

    # --- CameraSubmodel ---
    camera = []
    for l in root.findall("link"):
        if any(k in l.attrib.get("name", "").lower() for k in ["camera", "optical"]):
            elements = [create_property("type", "camera", prefix)]
            camera.append(create_collection(l.attrib.get("name"), elements, prefix))

    # --- MobileBaseSubmodel ---
    mobile_base = []
    for l in root.findall("link"):
        if "base" in l.attrib.get("name", "").lower():
            elements = [create_property("type", "mobile_base", prefix)]
            mobile_base.append(create_collection(l.attrib.get("name"), elements, prefix))

    # --- VisualizationSubmodel ---
    visualization = []
    for l in root.findall("link"):
        vis = l.find("visual")
        if vis is not None:
            geom = vis.find("geometry")
            mesh = geom.find("mesh") if geom is not None else None
            if mesh is not None:
                elements = [
                    create_property("mesh_file", mesh.attrib.get("filename"), prefix),
                    create_property("scale", mesh.attrib.get("scale", "1 1 1"), prefix)
                ]
                visualization.append(create_collection(l.attrib.get("name"), elements, prefix))

    submodels = {
        "StructureSubmodel": structure,
        "KinematicsSubmodel": kinematics,
        "DynamicsSubmodel": dynamics,
        "ControlSubmodel": control,
        "SafetySubmodel": safety,
        "EndEffectorSubmodel": endeffector,
        "CameraSubmodel": camera,
        "MobileBaseSubmodel": mobile_base,
        "VisualizationSubmodel": visualization
    }

    for name, elems in submodels.items():
        data = make_submodel(name, elems)
        out_path = os.path.join(output_dir, f"igus_rebel.{name}.json")
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)

    print(f"✅ 生成完成：所有 AAS Submodel JSON 文件已保存到 {output_dir}")

if __name__ == "__main__":
    generate_aas_submodels_rebel()
