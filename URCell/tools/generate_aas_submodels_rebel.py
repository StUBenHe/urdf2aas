#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, json, sys, pathlib, uuid

BASE = pathlib.Path(__file__).resolve().parents[1]  # URCell/
URDF_PATH = BASE / "types" / "igus_rebel_description_ros2" / "urdf" / "igus_rebel.urdf"
CONFIG_DIR = BASE / "types" / "config" / "igus_rebel"
OUT_DIR = BASE / "types" / "submodel" / "igus_rebel"

try:
    import yaml
except Exception:
    yaml = None
try:
    from urdfpy import URDF
except Exception:
    URDF = None

def load_yaml(name):
    path = CONFIG_DIR / name
    if not path.exists() or yaml is None:
        return {}
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}

def safe_urdf_load(path):
    if URDF is None or not path.exists():
        return None
    try:
        return URDF.load(str(path))
    except Exception:
        return None

def sm_header(id_short, desc):
    urn_base = f"urn:aas:igus:rebel:{uuid.uuid4()}"
    return {
        "modelType":"Submodel",
        "idShort": id_short,
        "id": f"{urn_base}:{id_short}",
        "category": "CONSTANT",
        "kind": "Instance",
        "administration": {"version":"0.1","revision":"0"},
        "description": [{"language":"en","text": desc}],
        "displayName": [{"language":"en","text": id_short}],
        "submodelElements": []
    }

def gen_structure(robot):
    sm = sm_header("StructureSubmodel", "Mechanical structure and composition of igus Rebel.")
    elems = []
    if robot:
        for link in robot.links:
            elems.append({"modelType":"Property","idShort":f"link::{link.name}","valueType":"string","value":"link"})
        for joint in robot.joints:
            elems.append({"modelType":"SubmodelElementCollection","idShort":f"joint::{joint.name}","value":[
                {"modelType":"Property","idShort":"type","valueType":"string","value":joint.joint_type},
                {"modelType":"Property","idShort":"parent","valueType":"string","value":str(joint.parent)},
                {"modelType":"Property","idShort":"child","valueType":"string","value":str(joint.child)}
            ]})
    else:
        elems.append({"modelType":"Property","idShort":"note","valueType":"string","value":"URDF not loaded; placeholder structure."})
    sm["submodelElements"] = elems
    return sm

def gen_control(robot, joint_limits_yaml):
    sm = sm_header("ControlSubmodel", "Controllers and joint limits for igus Rebel.")
    elems = []
    joints = robot.joints if robot else []
    for j in joints:
        lim = getattr(j, "limit", None)
        jl = joint_limits_yaml.get(getattr(j, "name", ""), {})
        elems.append({"modelType":"SubmodelElementCollection","idShort":f"joint::{j.name}::control","value":[
            {"modelType":"Property","idShort":"lower","valueType":"string","value":str(getattr(lim, "lower", jl.get("lower")))},
            {"modelType":"Property","idShort":"upper","valueType":"string","value":str(getattr(lim, "upper", jl.get("upper")))},
            {"modelType":"Property","idShort":"velocity","valueType":"string","value":str(getattr(lim, "velocity", jl.get("velocity")))},
            {"modelType":"Property","idShort":"effort","valueType":"string","value":str(getattr(lim, "effort", jl.get("effort")))}
        ]})
    sm["submodelElements"] = elems
    return sm

def gen_kinematics(kin_yaml):
    sm = sm_header("KinematicsSubmodel", "Kinematic parameters and frames.")
    elems = []
    for name, params in (kin_yaml or {}).items():
        elems.append({"modelType":"SubmodelElementCollection","idShort":f"kin::{name}","value":[
            *[{"modelType":"Property","idShort":k,"valueType":"string","value":str(v)} for k,v in (params or {}).items()]
        ]})
    sm["submodelElements"] = elems
    return sm

def gen_dynamics(robot, phys_yaml):
    sm = sm_header("DynamicsSubmodel", "Link inertial parameters.")
    elems = []
    if robot:
        for link in robot.links:
            inertial = getattr(link, "inertial", None)
            dyn = {"mass": None, "inertia": None, "origin": None}
            if inertial is not None:
                try:
                    dyn["mass"] = getattr(inertial, "mass", None)
                    I = getattr(inertial, "inertia", None)
                    dyn["inertia"] = I.tolist() if I is not None else None
                    O = getattr(inertial, "origin", None)
                    dyn["origin"] = O.tolist() if O is not None else None
                except Exception:
                    pass
            fallback = (phys_yaml.get("links", {})).get(link.name, {})
            for k in ("mass","inertia","origin"):
                if dyn[k] is None and k in fallback:
                    dyn[k] = fallback[k]
            elems.append({"modelType":"SubmodelElementCollection","idShort":f"dynamics::{link.name}","value":[
                *[{"modelType":"Property","idShort":k,"valueType":"string","value":str(v)} for k,v in dyn.items()]
            ]})
    sm["submodelElements"] = elems
    return sm

def gen_safety(robot):
    sm = sm_header("SafetySubmodel", "Safety-related limits derived from joint constraints.")
    elems = []
    if robot:
        for j in robot.joints:
            lim = getattr(j, "limit", None)
            if lim:
                elems.append({"modelType":"SubmodelElementCollection","idShort":f"safety::{j.name}","value":[
                    {"modelType":"Property","idShort":"lower","valueType":"string","value":str(getattr(lim,"lower", None))},
                    {"modelType":"Property","idShort":"upper","valueType":"string","value":str(getattr(lim,"upper", None))}
                ]})
    sm["submodelElements"] = elems
    return sm

def gen_visualization(robot, vis_yaml):
    sm = sm_header("VisualizationSubmodel", "Visualization assets for RViz/Sim.")
    elems = []
    if robot:
        for link in robot.links:
            visuals = getattr(link, "visuals", []) or []
            for idx, vis in enumerate(visuals):
                mesh_path = None
                try:
                    geom = getattr(vis, "geometry", None)
                    mesh = getattr(geom, "mesh", None) if geom else None
                    mesh_path = getattr(mesh, "filename", None) if mesh else None
                except Exception:
                    pass
                elems.append({"modelType":"SubmodelElementCollection","idShort":f"visual::{link.name}::{idx}","value":[
                    {"modelType":"Property","idShort":"mesh","valueType":"string","value":str(mesh_path)},
                ]})
    for link_name, params in (vis_yaml or {}).items():
        elems.append({"modelType":"SubmodelElementCollection","idShort":f"visual_override::{link_name}","value":[
            *[{"modelType":"Property","idShort":k,"valueType":"string","value":str(v)} for k,v in (params or {}).items()]
        ]})
    sm["submodelElements"] = elems
    return sm

def gen_camera(cam_yaml):
    sm = sm_header("CameraSubmodel", "Camera modules and virtual frames.")
    assets = (cam_yaml or {}).get("assets", [])
    elems = [
        {"modelType":"Property","idShort":"default_mount","valueType":"string","value":str((cam_yaml or {}).get("default_mount","tool0"))},
        {"modelType":"SubmodelElementCollection","idShort":"assets","value":[
            *[{"modelType":"Property","idShort":f"asset::{i}","valueType":"string","value":str(p)} for i,p in enumerate(assets)]
        ]}
    ]
    sm["submodelElements"] = elems
    return sm

def gen_end_effector(ee_yaml):
    sm = sm_header("EndEffectorSubmodel", "End-effector variants (soft gripper, toucher, etc.).")
    variants = (ee_yaml or {}).get("variants", [])
    elems = [
        {"modelType":"Property","idShort":"tool_mount","valueType":"string","value":"tool0"},
        {"modelType":"SubmodelElementCollection","idShort":"variants","value":[
            *[{"modelType":"Property","idShort":f"variant::{i}","valueType":"string","value":str(p)} for i,p in enumerate(variants)]
        ]}
    ]
    sm["submodelElements"] = elems
    return sm

def gen_mobile_base(mb_yaml):
    sm = sm_header("MobileBaseSubmodel", "Mobile base stack (castle).")
    assets = (mb_yaml or {}).get("assets", [])
    elems = [
        {"modelType":"Property","idShort":"attach_to","valueType":"string","value":str((mb_yaml or {}).get("attach_to","base_link"))},
        {"modelType":"SubmodelElementCollection","idShort":"assets","value":[
            *[{"modelType":"Property","idShort":f"asset::{i}","valueType":"string","value":str(p)} for i,p in enumerate(assets)]
        ]}
    ]
    sm["submodelElements"] = elems
    return sm

def save_json(obj, name):
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    path = OUT_DIR / name
    with open(path, "w", encoding="utf-8") as f:
        json.dump(obj, f, indent=2, ensure_ascii=False)
    return str(path)

def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    robot = safe_urdf_load(URDF_PATH)
    joint_limits = load_yaml("joint_limits.yaml")
    kin = load_yaml("default_kinematics.yaml")
    phys = load_yaml("physical_parameters.yaml")
    vis = load_yaml("visual_parameters.yaml")
    cam = load_yaml("camera_parameters.yaml")
    ee = load_yaml("end_effector_parameters.yaml")
    mb = load_yaml("mobile_base_parameters.yaml")

    artifacts = []
    artifacts.append(save_json(gen_structure(robot), "igus_rebel.StructureSubmodel.json"))
    artifacts.append(save_json(gen_control(robot, joint_limits), "igus_rebel.ControlSubmodel.json"))
    artifacts.append(save_json(gen_kinematics(kin), "igus_rebel.KinematicsSubmodel.json"))
    artifacts.append(save_json(gen_dynamics(robot, phys), "igus_rebel.DynamicsSubmodel.json"))
    artifacts.append(save_json(gen_safety(robot), "igus_rebel.SafetySubmodel.json"))
    artifacts.append(save_json(gen_visualization(robot, vis), "igus_rebel.VisualizationSubmodel.json"))
    artifacts.append(save_json(gen_camera(cam), "igus_rebel.CameraSubmodel.json"))
    artifacts.append(save_json(gen_end_effector(ee), "igus_rebel.EndEffectorSubmodel.json"))
    artifacts.append(save_json(gen_mobile_base(mb), "igus_rebel.MobileBaseSubmodel.json"))

    print("Generated files:")
    for a in artifacts:
        print("-", a)

if __name__ == "__main__":
    main()
