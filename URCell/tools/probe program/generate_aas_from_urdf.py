import os
import json
import yaml
import re
from lxml import etree
from urdfpy import URDF
import numpy as np
import uuid
import os
import json
from pathlib import Path

def _guess_value_type(v):
    if isinstance(v, bool):
        return "xs:boolean"
    if isinstance(v, int) or isinstance(v, float):
        return "xs:double"
    # 其它一律存成字符串
    return "xs:string"

def _to_property(id_short, v):
    # 列表/字典 -> 序列化存成字符串，避免再次触发“期望数组”的错误
    if isinstance(v, (list, tuple, dict)):
        value = json.dumps(v, ensure_ascii=False)
        value_type = "xs:string"
    else:
        value = str(v)
        value_type = _guess_value_type(v)
    return {
        "modelType": "Property",
        "idShort": id_short,
        "valueType": value_type,
        "value": value
    }

def _to_file(id_short, path_str):
    # 依据扩展名给出一个合理的 contentType
    ext = Path(path_str).suffix.lower()
    mime = {
        ".stl": "model/stl",
        ".dae": "model/vnd.collada+xml",
        ".obj": "model/obj",
        ".glb": "model/gltf-binary",
        ".gltf": "model/gltf+json"
    }.get(ext, "application/octet-stream")
    return {
        "modelType": "File",
        "idShort": id_short,
        "contentType": mime,
        "value": path_str
    }

def _kv_to_smc_value(obj):
    """
    把 dict 转为 AAS 合法的 SMC.value（数组）
    规则：
      - "mesh" -> File
      - 其它键：
          - 标量 -> Property
          - 列表/字典 -> Property(字符串化) 以保证 AASX 不再要求子层也是数组
    """
    items = []
    for k, v in obj.items():
        if k == "mesh" and isinstance(v, str) and v:
            items.append(_to_file("mesh", v))
        else:
            items.append(_to_property(k, v))
    return items


def fix_submodel_structure(node):
    """递归修正所有 SubmodelElementCollection 的 value 为 list，避免空对象。"""
    if isinstance(node, dict):
        # 修正当前 SMC
        if node.get("modelType") == "SubmodelElementCollection":
            val = node.get("value")
            if isinstance(val, dict):
                node["value"] = [
                    {
                        "modelType": "Property",
                        "idShort": k,
                        "valueType": "xs:string" if not isinstance(v, (int, float)) else "xs:double",
                        "value": json.dumps(v, ensure_ascii=False)
                        if isinstance(v, (list, dict)) else str(v)
                    }
                    for k, v in val.items()
                    if v not in [None, {}, []]  # 跳过空值
                ]
        # 递归子项
        for v in node.values():
            fix_submodel_structure(v)
    elif isinstance(node, list):
        for v in node:
            fix_submodel_structure(v)


def make_paths_relative(node, base_marker="ur_description"):
    """
    递归扫描所有 mesh 字段，把绝对路径改成以 package://ur_description/... 为前缀的相对路径
    """
    if isinstance(node, dict):
        for k, v in list(node.items()):
            if isinstance(v, str) and base_marker in v:
                # 截取 ur_description 及其后面的路径部分
                relative_part = v.split(base_marker, 1)[-1].lstrip("/\\")
                node[k] = f"package://{base_marker}/{relative_part}"
            else:
                make_paths_relative(v, base_marker)
    elif isinstance(node, list):
        for item in node:
            make_paths_relative(item, base_marker)

def _fix_smc_value_inplace(node):
    """
    递归修正：凡是 modelType == 'SubmodelElementCollection' 的元素，
    若 value 不是 list，则改造成 list；子层继续递归。
    """
    if isinstance(node, dict):
        # 修正当前层
        if node.get("modelType") == "SubmodelElementCollection":
            val = node.get("value")
            if isinstance(val, dict):          # 核心修复：dict -> list
                node["value"] = _kv_to_smc_value(val)
            elif not isinstance(val, list):    # None 或标量 -> 包一层 Property
                node["value"] = [_to_property("value", val)]
        # 递归子字段
        for k, v in list(node.items()):
            _fix_smc_value_inplace(v)
    elif isinstance(node, list):
        for it in node:
            _fix_smc_value_inplace(it)

def _wrap_as_submodel(content, base_name, fallback_idshort):
    """
    把 {idShort, submodelElements: ...} 包成 AASX 可识别的 Submodel 对象。
    并统一把所有 SMC 的 value 修正成数组。
    """
    # 1) 确保有 idShort 与 submodelElements
    id_short = content.get("idShort", fallback_idshort)
    sm_elements = content.get("submodelElements", [])

    # 2) 递归修正所有 SMC.value
    _fix_smc_value_inplace(sm_elements)

    # 3) 封装成 Submodel（AASX 接受 "modelType": "Submodel" 这一写法）
    return {
        "modelType": "Submodel",
        "id": f"urn:submodel:robot:{id_short}:{base_name}:safe",
        "idShort": id_short,
        "category": f"Robot:{base_name.upper()}",
        "submodelElements": sm_elements
    }

# === 自定义 YAML 解析器，支持 ROS 风格的 !degrees ===
def degrees_constructor(loader, node):
    value = loader.construct_scalar(node)
    try:
        return float(value)
    except ValueError:
        return value
yaml.SafeLoader.add_constructor('!degrees', degrees_constructor)

# ====== 修复 numpy 兼容性问题 ======
if not hasattr(np, 'float'):
    np.float = float
# ==================================

# ===============================
# 📘 工具函数
# ===============================
def load_yaml(file_path):
    if not os.path.exists(file_path):
        return {}
    with open(file_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)

def save_submodel(submodel, name, output_dir):
    os.makedirs(output_dir, exist_ok=True)
    file_path = os.path.join(output_dir, name + ".json")
    with open(file_path, "w", encoding="utf-8") as f:
        json.dump(submodel, f, indent=2, ensure_ascii=False)
    print(f"💾 已生成: {file_path}")

def safe_list(value):
    if value is None:
        return None
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (list, tuple)):
        return list(value)
    return value


# ===============================
# 🧩 子模型生成函数
# ===============================
def generate_structure_submodel(robot):
    """从 URDF 模型生成完整结构子模型（包含 link 与 joint 的实际数值）"""
    elements = []

    # === 解析 Link ===
    for link in robot.links:
        link_dict = {"type": "link"}
        if link.inertial:
            inertial = link.inertial
            link_dict["mass"] = inertial.mass
            if inertial.origin is not None:
                link_dict["com"] = safe_list(inertial.origin)
            if inertial.inertia is not None:
                link_dict["inertia"] = safe_list(inertial.inertia)

        visuals = []
        for vis in link.visuals:
            mesh_file = None
            if hasattr(vis.geometry, "mesh") and vis.geometry.mesh:
                mesh_file = vis.geometry.mesh.filename
            visuals.append({
                "mesh": mesh_file,
                "origin": safe_list(vis.origin)
            })
        if visuals:
            link_dict["visuals"] = visuals

        collisions = []
        for col in link.collisions:
            mesh_file = None
            if hasattr(col.geometry, "mesh") and col.geometry.mesh:
                mesh_file = col.geometry.mesh.filename
            collisions.append({
                "mesh": mesh_file,
                "origin": safe_list(col.origin)
            })
        if collisions:
            link_dict["collisions"] = collisions

        elements.append({
            "idShort": f"Link_{link.name}",
            "modelType": "SubmodelElementCollection",
            "value": link_dict
        })

    # === 解析 Joint ===
    for joint in robot.joints:
        joint_dict = {
            "type": joint.joint_type,
            "parent": joint.parent,
            "child": joint.child
        }

        if joint.origin is not None:
            origin_data = safe_list(joint.origin)
            if isinstance(origin_data, (list, tuple)) and len(origin_data) >= 2:
                joint_dict["origin"] = {
                    "xyz": safe_list(origin_data[0]),
                    "rpy": safe_list(origin_data[1])
                }

        if hasattr(joint, "axis") and joint.axis is not None:
            joint_dict["axis"] = safe_list(joint.axis)

        if hasattr(joint, "limit") and joint.limit is not None:
            limit = joint.limit
            joint_dict["limit"] = {
                "lower": limit.lower,
                "upper": limit.upper,
                "effort": limit.effort,
                "velocity": limit.velocity
            }

        if hasattr(joint, "dynamics") and joint.dynamics is not None:
            dyn = joint.dynamics
            joint_dict["dynamics"] = {
                "damping": getattr(dyn, "damping", None),
                "friction": getattr(dyn, "friction", None)
            }

        elements.append({
            "idShort": f"Joint_{joint.name}",
            "modelType": "SubmodelElementCollection",
            "value": joint_dict
        })

    return {"idShort": "StructureSubmodelFull", "submodelElements": elements}


def generate_control_submodel(robot, control_template_path, joint_limits):
    """从 URDF + 控制模板（JSON）生成完整控制子模型"""
    controllers_data = {}
    if os.path.exists(control_template_path):
        with open(control_template_path, "r", encoding="utf-8") as f:
            control_template = json.load(f)
        for sm_element in control_template.get("submodelElements", []):
            if sm_element["idShort"] == "Controllers":
                for controller in sm_element["value"]:
                    ctrl_id = controller["idShort"]
                    ctrl_params = {}
                    for v in controller["value"]:
                        if v["idShort"] == "mode":
                            ctrl_params["mode"] = v["value"]
                        elif v["idShort"] == "updateRateHz":
                            ctrl_params["updateRateHz"] = float(v["value"])
                        elif v["idShort"] == "gains":
                            ctrl_params["gains"] = {g["idShort"]: float(g["value"]) for g in v["value"]}
                        elif v["idShort"] == "targets":
                            ctrl_params["targets"] = [t["value"] for t in v["value"]]
                    controllers_data[ctrl_id] = ctrl_params

    elements = []
    for joint in robot.joints:
        limit = joint.limit
        limits_yaml = joint_limits.get(joint.name, {})
        joint_entry = {
            "lower": limit.lower if limit else limits_yaml.get("lower"),
            "upper": limit.upper if limit else limits_yaml.get("upper"),
            "effort": limit.effort if limit else limits_yaml.get("effort"),
            "velocity": limit.velocity if limit else limits_yaml.get("velocity"),
        }

        # 控制器匹配
        for ctrl_id, params in controllers_data.items():
            if joint.name in params.get("targets", []):
                joint_entry.update({
                    "controller_id": ctrl_id,
                    "mode": params.get("mode"),
                    "updateRateHz": params.get("updateRateHz"),
                    "gains": params.get("gains")
                })

        elements.append({
            "idShort": f"Joint_{joint.name}_Control",
            "modelType": "SubmodelElementCollection",
            "value": joint_entry
        })

    return {"idShort": "ControlSubmodelFull", "submodelElements": elements}


def generate_kinematics_submodel(kinematics_yaml):
    elements = []
    for joint_name, params in kinematics_yaml.items():
        elements.append({
            "idShort": f"Kinematics_{joint_name}",
            "modelType": "SubmodelElementCollection",
            "value": params
        })
    return {"idShort": "KinematicsSubmodel", "submodelElements": elements}


def generate_dynamics_submodel(robot, physical_params):
    elements = []
    for link in robot.links:
        inertial = link.inertial
        params_yaml = physical_params.get(link.name, {})
        dynamics = {
            "mass": inertial.mass if inertial else params_yaml.get("mass"),
            "inertia": safe_list(inertial.inertia if inertial else params_yaml.get("inertia")),
            "origin": safe_list(inertial.origin if inertial else params_yaml.get("origin"))
        }
        elements.append({
            "idShort": f"Dynamics_{link.name}",
            "modelType": "SubmodelElementCollection",
            "value": dynamics
        })
    return {"idShort": "DynamicsSubmodel", "submodelElements": elements}


def generate_safety_submodel(robot):
    elements = []
    for joint in robot.joints:
        if joint.limit:
            elements.append({
                "idShort": f"Joint_{joint.name}_Safety",
                "modelType": "SubmodelElementCollection",
                "value": {"lower": joint.limit.lower, "upper": joint.limit.upper}
            })
    return {"idShort": "SafetySubmodel", "submodelElements": elements}


def generate_visualization_submodel(robot, visual_params):
    elements = []
    for link in robot.links:
        vis_param = visual_params.get(link.name, {})
        for i, vis in enumerate(link.visuals):
            mesh_file = None
            if hasattr(vis.geometry, "mesh") and vis.geometry.mesh:
                mesh_file = vis.geometry.mesh.filename
            elements.append({
                "idShort": f"Visual_{link.name}_{i}",
                "modelType": "SubmodelElementCollection",
                "value": {
                    "mesh": mesh_file,
                    "color": vis_param.get("color"),
                    "material": vis_param.get("material")
                }
            })
    return {"idShort": "VisualizationSubmodel", "submodelElements": elements}

import uuid
import json
import os

def assemble_environment_v3(base_name, output_dir):
    """
    汇总所有子模型并修正结构，每个子模型带上标签前缀（如 UR3_Structure）。
    生成文件可直接通过 “Import Submodel from JSON” 导入。
    """
    submodel_files = [
        f"{base_name}_structure_submodel.json",
        f"{base_name}_control_submodel.json",
        f"{base_name}_kinematics_submodel.json",
        f"{base_name}_dynamics_submodel.json",
        f"{base_name}_safety_submodel.json",
        f"{base_name}_visualization_submodel.json",
    ]

    submodels = []
    base_upper = base_name.upper()

    for file_name in submodel_files:
        path = os.path.join(output_dir, file_name)
        if not os.path.exists(path):
            print(f"⚠️ 缺少子模型文件: {file_name}")
            continue

        with open(path, "r", encoding="utf-8") as f:
            content = json.load(f)

        # 🧩 自动修复内部结构
        fix_submodel_structure(content)

        # 提取子模型类型标签，例如 "structure"
        label_part = file_name.replace(f"{base_name}_", "").replace("_submodel.json", "")
        label_upper = label_part.capitalize()

        # 🔖 拼接成标签名
        id_short = f"{base_upper}_{label_upper}Submodel"
        sm_id = f"urn:submodel:robot:{base_upper}:{label_upper}"

        submodels.append({
            "modelType": "Submodel",
            "id": sm_id,
            "idShort": id_short,
            "category": f"Robot:{base_upper}",
            "submodelElements": content.get("submodelElements", [])
        })

    environment = {"submodels": submodels}
    # ✅ 在写入文件之前执行路径相对化修正
    make_paths_relative(environment)

    output_path = os.path.join(output_dir, f"{base_name}_environment.json")

    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(environment, f, indent=2, ensure_ascii=False)

    print(f"✅ 已生成带标签的可导入文件: {output_path}")


import argparse, json, re, subprocess, sys
from pathlib import Path
from typing import Any, Dict, List, Optional

# =========================
# 语义映射（semanticId）
# =========================
URN_MAP = {
    "structure": "urn:robot:submodel:Structure:1:0",
    "kinematics": "urn:robot:submodel:Kinematics:1:0",
    "dynamics": "urn:robot:submodel:Dynamics:1:0",
    "control": "urn:robot:submodel:Control:1:0",
    "visualization": "urn:robot:submodel:Visualization:1:0",
    "safety": "urn:robot:submodel:Safety:1:0",
    "environment": "urn:robot:submodel:Environment:1:0",
}
PAT = re.compile("|".join(sorted(map(re.escape, URN_MAP.keys()), key=len, reverse=True)), re.I)


# =========================
# 工具函数
# =========================
def infer_category(p: Path) -> Optional[str]:
    text = (p.parent.name + "_" + p.stem).lower()
    m = PAT.search(text)
    return m.group(0).lower() if m else None

def make_semantic(urn: str) -> Dict[str, Any]:
    return {"type": "ExternalReference", "keys": [{"type": "GlobalReference", "value": urn}]}

def normalize_idshort(s: str) -> str:
    s2 = re.sub(r'[^A-Za-z0-9_]', '_', s).strip('_')
    if not s2 or not re.match(r'[A-Za-z]', s2[0]):
        s2 = "AAS_" + s2
    return s2

def is_submodel_dict(d: Any) -> bool:
    if not isinstance(d, dict):
        return False
    mt = d.get("modelType")
    # 兼容：缺失 modelType 也视为 Submodel（向后兼容）
    return (mt is None) or (mt == "Submodel")


# =========================
# Submodel 修复
# =========================
def fix_submodel(path: Path, kind_value: str, force: bool, write: bool) -> Optional[Dict[str, Any]]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception as e:
        print(f"  ⏭️  跳过解析失败 {path.name}: {e}")
        return None

    if not is_submodel_dict(data):
        return None

    changed = False

    # 确保 v3 风格
    if data.get("modelType") != "Submodel":
        data["modelType"] = "Submodel"
        changed = True

    # kind
    if data.get("kind") != kind_value:
        data["kind"] = kind_value
        changed = True

    # semanticId
    sem = data.get("semanticId")
    need_set = force or (not isinstance(sem, dict) or not sem.get("keys") or not sem["keys"][0].get("value"))
    if need_set:
        cat = infer_category(path)
        urn = URN_MAP.get(cat, f"urn:robot:submodel:{path.stem}:1:0")
        data["semanticId"] = make_semantic(urn)
        changed = True

    if changed and write:
        path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")

    print(f"  {'✔️ 更新' if changed else '⏩ 保持'}: {path.name}  kind={data.get('kind')}  semanticId={data['semanticId']['keys'][0]['value']}")
    return data


# =========================
# 组装 AAS 环境
# =========================
def build_aas_environment(model_name: str, submodels: List[Dict[str, Any]], kind_value: str) -> Dict[str, Any]:
    aas_id = f"urn:robot:aas:{model_name.upper()}_001"
    aas_idshort = f"{model_name.upper()}_Shell"

    env = {
        "assetAdministrationShells": [{
            "id": aas_id,
            "idShort": aas_idshort,
            "assetInformation": {
                "assetKind": kind_value,
                "kind": kind_value,
                "globalAssetId": f"urn:robot:asset:{model_name.upper()}"
            },
            "submodels": [
                {
                    "type": "ModelReference",
                    "keys": [{
                        "type": "Submodel",
                        "value": sm.get("id", f"urn:robot:submodel:{model_name}:{i}")
                    }]
                } for i, sm in enumerate(submodels)
            ]
        }],
        "submodels": submodels,
        "conceptDescriptions": []  # 按你的要求保留空数组
    }
    return env


# =========================
# 生成 submodels：封装 generate_aas_submodels_ur
# =========================
def try_import_and_call_generator(urdf_path: Path, out_dir: Path) -> bool:
    """
    优先尝试 import tools.generate_aas_submodels_ur 并调用 generate_submodels(urdf_path, out_dir)。
    返回 True 表示已成功调用。
    """
    try:
        sys.path.insert(0, str(Path.cwd()))  # 确保以项目根为基准可导入 tools 包
        import tools.generate_aas_submodels_ur as gen  # type: ignore
        if hasattr(gen, "generate_submodels"):
            print(f"  🧰 调用: tools.generate_aas_submodels_ur.generate_submodels({urdf_path}, {out_dir})")
            gen.generate_submodels(str(urdf_path), str(out_dir))  # 统一用 str
            return True
        else:
            print("  ⚠️ 模块已导入，但未找到函数 generate_submodels(...)，将尝试子进程方式。")
            return False
    except Exception as e:
        print(f"  ⚠️ 导入 generate_aas_submodels_ur 失败: {e}，将尝试子进程方式。")
        return False

def run_generator_subprocess(urdf_path: Path, out_dir: Path) -> None:
    """
    回退方案：用 subprocess 运行 tools/generate_aas_submodels_ur.py。
    兼容两种 CLI 习惯：--urdf/--out 或 -i/-o。
    """
    script = Path("tools") / "generate_aas_submodels_ur.py"
    if not script.exists():
        raise FileNotFoundError(f"未找到脚本: {script}")

    tried = []

    # 优先长参数
    cmd1 = [sys.executable, str(script), "--urdf", str(urdf_path), "--out", str(out_dir)]
    tried.append(cmd1)
    # 备选短参数
    cmd2 = [sys.executable, str(script), "-i", str(urdf_path), "-o", str(out_dir)]
    tried.append(cmd2)

    for cmd in tried:
        print("  🧰 子进程调用：", " ".join(cmd))
        proc = subprocess.run(cmd, capture_output=True, text=True)
        if proc.returncode == 0:
            if proc.stdout:
                print(proc.stdout.strip())
            if proc.stderr:
                print(proc.stderr.strip())
            return
        else:
            print(f"  ⚠️ 子进程返回码 {proc.returncode}")
            if proc.stdout:
                print(proc.stdout.strip())
            if proc.stderr:
                print(proc.stderr.strip())

    raise RuntimeError("无法成功调用 generate_aas_submodels_ur.py（已尝试 --urdf/--out 与 -i/-o 两种参数样式）。")

def ensure_submodels(robot_dir: Path, robot_name: str, urdf_hint: Optional[Path], regen: bool) -> None:
    """
    如果 robot_dir 下没有有效 submodels，或 regen=True，则从 URDF 生成。
    urdf_hint 指定时直用；否则自动按常见路径探测。
    """
    need_generate = regen or not any(robot_dir.glob("*.json"))

    if not need_generate:
        # 目录里已有 .json，简单检查是否至少一个能解析为 Submodel
        for f in robot_dir.glob("*.json"):
            try:
                data = json.loads(f.read_text(encoding="utf-8"))
                if is_submodel_dict(data):
                    return  # 有效，直接返回
            except Exception:
                continue
        need_generate = True

    if not need_generate:
        return

    # 需要生成：定位 URDF
    urdf_path: Optional[Path] = None
    if urdf_hint:
        urdf_path = urdf_hint if urdf_hint.exists() else None
    if not urdf_path:
        # 常见候选位置（相对项目根）
        cands = [
            Path("types/ur_description/urdf") / f"{robot_name}.urdf",
            Path("types/ur_description/urdf/pythonProject") / f"{robot_name}.urdf",
        ]
        for c in cands:
            if c.exists():
                urdf_path = c
                break

    if not urdf_path:
        raise FileNotFoundError(
            f"未找到 {robot_name} 对应的 URDF。可使用 --from-urdf 指定，或将 urdf 放在:\n"
            f"  types/ur_description/urdf/{robot_name}.urdf\n"
            f"  types/ur_description/urdf/pythonProject/{robot_name}.urdf"
        )

    robot_dir.mkdir(parents=True, exist_ok=True)

    print(f"  🔧 从 URDF 生成 submodels: {urdf_path} -> {robot_dir}")
    # 先尝试 import 调用；失败则用子进程
    if not try_import_and_call_generator(urdf_path, robot_dir):
        run_generator_subprocess(urdf_path, robot_dir)


# =========================
# 处理单个机器人目录
# =========================
def process_robot_dir(robot_dir: Path, kind_value: str, force: bool, force_idshort: bool,
                      write: bool, urdf_hint: Optional[Path], regen: bool):
    model_name = robot_dir.name
    print(f"\n📂 正在处理: {model_name}")

    # 需要时先生成 submodels
    try:
        ensure_submodels(robot_dir, model_name, urdf_hint, regen)
    except Exception as e:
        print(f"  ❌ 生成 submodels 失败: {e}")
        return

    # 修复 submodels
    submodels: List[Dict[str, Any]] = []
    for f in sorted(robot_dir.glob("*.json")):
        sm = fix_submodel(f, kind_value, force, write)
        if sm:
            submodels.append(sm)

    if not submodels:
        print(f"  ⚠️ 未找到有效 Submodel 文件: {model_name}")
        return

    # 组装 v3 环境
    env = build_aas_environment(model_name, submodels, kind_value)

    # idShort 修复（可覆盖）
    shell = env["assetAdministrationShells"][0]
    if force_idshort or not shell.get("idShort"):
        shell["idShort"] = normalize_idshort(model_name)

    # 写出
    output_path = robot_dir / f"{model_name}_environment_v3.json"
    if write:
        output_path.write_text(json.dumps(env, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"✅ 已生成: {output_path.name}")

def ensure_value_is_list(node):
    """
    递归修正所有 SubmodelElementCollection 的 value，
    将 dict -> list，确保符合 AAS v3 格式。
    """
    if isinstance(node, dict):
        if node.get("modelType") == "SubmodelElementCollection":
            val = node.get("value")
            if isinstance(val, dict):
                # 转换为 list
                node["value"] = list(val.values())
        # 递归遍历子项
        for v in node.values():
            ensure_value_is_list(v)
    elif isinstance(node, list):
        for v in node:
            ensure_value_is_list(v)



# ===============================
# ⚙️ 主逻辑
# ===============================
def main(urdf_path, yaml_dir, output_dir="output"):
    print(f"\n=== Generating AAS Submodels for {os.path.basename(urdf_path).split('.')[0]} ===")

    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    urcell_root = base_dir.replace("\\", "/")
    package_root = os.path.join(urcell_root, "types", "ur_description").replace("\\", "/")

    with open(urdf_path, "r", encoding="utf-8") as f:
        urdf_text = f.read().replace("\\", "/")
    urdf_text = urdf_text.replace("package://ur_description/", package_root + "/").replace("//", "/")

    if "package://" in urdf_text:
        print("⚠️ 部分 package:// 路径未替换（保留 ROS 兼容格式）。")
    else:
        print("✅ 路径解析成功，所有 mesh 均可解析。")

    debug_dir = os.path.join(os.path.dirname(__file__), "output")
    os.makedirs(debug_dir, exist_ok=True)
    debug_path = os.path.join(debug_dir, f"debug_{os.path.basename(urdf_path).split('.')[0]}_fixed.urdf")
    with open(debug_path, "w", encoding="utf-8") as f:
        f.write(urdf_text)
    print(f"📄 已输出调试文件: {debug_path}")

    parser = etree.XMLParser(remove_blank_text=True)
    root = etree.fromstring(urdf_text.encode("utf-8"), parser=parser)
    robot = URDF._from_xml(root, None)
    print(f"🤖 成功加载 URDF 模型: {robot.name}, 包含 {len(robot.links)} 个 link, {len(robot.joints)} 个 joint。")

    joint_limits = load_yaml(os.path.join(yaml_dir, "joint_limits.yaml"))
    kinematics = load_yaml(os.path.join(yaml_dir, "default_kinematics.yaml"))
    physical = load_yaml(os.path.join(yaml_dir, "physical_parameters.yaml"))
    visual = load_yaml(os.path.join(yaml_dir, "visual_parameters.yaml"))
    base_name = os.path.splitext(os.path.basename(urdf_path))[0]
    control_template_path = os.path.join(yaml_dir, f"{base_name}_control.json")

    save_submodel(generate_structure_submodel(robot), f"{base_name}_structure_submodel", output_dir)
    save_submodel(generate_control_submodel(robot, control_template_path, joint_limits), f"{base_name}_control_submodel", output_dir)
    save_submodel(generate_kinematics_submodel(kinematics), f"{base_name}_kinematics_submodel", output_dir)
    save_submodel(generate_dynamics_submodel(robot, physical), f"{base_name}_dynamics_submodel", output_dir)
    save_submodel(generate_safety_submodel(robot), f"{base_name}_safety_submodel", output_dir)
    save_submodel(generate_visualization_submodel(robot, visual), f"{base_name}_visualization_submodel", output_dir)
    assemble_environment_v3(base_name, output_dir)

    print(f"🎯 已完成 {base_name} 的所有 AAS 子模型生成。\n")



if __name__ == "__main__":
    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    urdf_root = os.path.join(base_dir, "types", "ur_description", "urdf")
    config_root = os.path.join(base_dir, "types", "ur_description", "config")
    submodel_root = os.path.join(base_dir, "types", "submodel")

    ur_names = [
        "ur3",
        "ur3e",
        "ur5",
        "ur5e",
        "ur7e",
        "ur10",
        "ur10e",
        "ur12e",
        "ur15",
        "ur16e",
        "ur20",
        "ur30"
    ]
    for name in ur_names:
        urdf_file = os.path.join(urdf_root, f"{name}.urdf")
        yaml_dir = os.path.join(config_root, name)
        output_dir = os.path.join(submodel_root, name)

        if not os.path.exists(urdf_file):
            print(f"❌ 未找到 URDF: {urdf_file}")
            continue
        if not os.path.exists(yaml_dir):
            print(f"⚠️ YAML 配置缺失: {yaml_dir}（将使用空默认值）")
            os.makedirs(yaml_dir, exist_ok=True)

        try:
            main(urdf_file, yaml_dir, output_dir)
        except Exception as e:
            print(f"❌ 处理 {name} 时出错: {e}")
if __name__ == "__main__":
    print("\n🚀 一键生成所有 UR 系列机器人的 AAS Environment v3 文件\n")

    base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    urdf_root = os.path.join(base_dir, "types", "ur_description", "urdf")
    config_root = os.path.join(base_dir, "types", "ur_description", "config")
    submodel_root = os.path.join(base_dir, "types", "submodel")

    # ✅ 批量支持的 UR 机器人列表
    ur_names = [
        "ur3", "ur3e", "ur5", "ur5e", "ur7e",
        "ur10", "ur10e", "ur12e", "ur15",
        "ur16e", "ur20", "ur30"
    ]

    for name in ur_names:
        print(f"\n==============================")
        print(f"🤖 处理 {name.upper()} 开始")
        print("==============================")

        urdf_file = os.path.join(urdf_root, f"{name}.urdf")
        yaml_dir = os.path.join(config_root, name)
        output_dir = os.path.join(submodel_root, name)

        # 检查 URDF
        if not os.path.exists(urdf_file):
            print(f"❌ 未找到 URDF 文件: {urdf_file}")
            continue

        # 检查 YAML
        if not os.path.exists(yaml_dir):
            print(f"⚠️ 未找到配置目录: {yaml_dir}，已自动创建空目录。")
            os.makedirs(yaml_dir, exist_ok=True)

        try:
            # === 第一步：从 URDF 生成各子模型 JSON ===
            print(f"📦 正在从 URDF 生成子模型 ...")
            main(urdf_file, yaml_dir, output_dir)

            # === 第二步：将子模型语义化并生成 Environment v3 ===
            print(f"🧩 正在生成 AAS Environment v3 ...")
            process_robot_dir(
                Path(output_dir),
                kind_value="Template",
                force=True,
                force_idshort=True,
                write=True,
                urdf_hint=Path(urdf_file),
                regen=False
            )

            print(f"✅ 已完成 {name.upper()} 的完整 Environment v3 生成。")

        except Exception as e:
            print(f"❌ 处理 {name} 时出错: {e}")

    print("\n🎯 所有 UR 系列机器人已处理完毕。输出目录：types/submodel/")
