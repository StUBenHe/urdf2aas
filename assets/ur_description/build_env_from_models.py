#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
从目录 aas/<model>/<model>_xxx_submodel.json 生成可被
AASX Package Explorer 直接导入的 AAS V3 Environment JSON。

示例目录：
aas/
  ur3/
    ur3_control_submodel.json
    ur3_dynamics_submodel.json
    ur3_kinematics_submodel.json
    ur3_safety_submodel.json
    ur3_structure_submodel.json
    ur3_visualization_submodel.json
  ur5/
    ur5_control_submodel.json
    ...

运行：
  python build_env_from_models.py

输出：
  out_env/ur3.env.json
  out_env/ur5.env.json
  ...
"""

import os, re, json, pathlib
from typing import Any, Dict, List

AAS_ROOT = "aas"
OUT_DIR  = "out_env"

# ---------- 小工具 ----------
def read_json(p: pathlib.Path) -> Any:
    with open(p, "r", encoding="utf-8") as f:
        return json.load(f)

def write_json(p: pathlib.Path, data: Any):
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

def idshort_safe(s: str) -> str:
    s = re.sub(r"[^\w]+", "_", s.strip())
    if not s or not re.match(r"^[A-Za-z_]", s):
        s = "x_" + s
    return s

def make_urn(prefix: str, *parts: str) -> str:
    parts = [idshort_safe(p).lower() for p in parts if p]
    return "urn:%s:%s" % (prefix, ":".join(parts))

def guess_value_type(v: Any) -> str:
    if isinstance(v, bool):  return "boolean"
    if isinstance(v, int):   return "int"
    if isinstance(v, float): return "double"
    return "string"

def stringify(v: Any) -> str:
    if v is None: return ""
    if isinstance(v, bool): return "true" if v else "false"
    return str(v)

# ---------- JSON → SubmodelElements ----------
def json_to_sme(name: str, data: Any) -> Dict[str, Any]:
    idshort = idshort_safe(name)
    if isinstance(data, dict):
        return {
            "modelType": "SubmodelElementCollection",
            "idShort": idshort,
            "value": [json_to_sme(k, v) for k, v in data.items()]
        }
    if isinstance(data, list):
        return {
            "modelType": "SubmodelElementCollection",
            "idShort": idshort,
            "value": [json_to_sme(f"idx_{i}", v) for i, v in enumerate(data)]
        }
    return {
        "modelType": "Property",
        "idShort": idshort,
        "valueType": guess_value_type(data),
        "value": stringify(data)
    }

# ---------- 构建 AAS 结构 ----------
def build_submodel(model: str, submodel_key: str, payload: Any) -> Dict[str, Any]:
    """
    model:  ur3 / ur5 …
    submodel_key: control / dynamics / kinematics / safety / structure / visualization
    payload: 原始 JSON，将放入顶层 SMC 'Payload'
    """
    sm_id  = make_urn("aas:submodel", model, submodel_key, "1-0")
    sm_sem = make_urn("sem", "robot", submodel_key)
    payload_smc = json_to_sme("Payload", payload)
    return {
        "modelType": "Submodel",
        "id": sm_id,
        "idShort": submodel_key.capitalize(),
        "kind": "Instance",  # 如果你想做模板库，改为 "Template"
        "semanticId": {
            "type": "GlobalReference",
            "keys": [ { "type": "GlobalReference", "value": sm_sem } ]
        },
        "submodelElements": [payload_smc]
    }

def build_aas(model: str, submodels: List[Dict[str, Any]]) -> Dict[str, Any]:
    aas_id = make_urn("aas", model)
    return {
        "modelType": "AssetAdministrationShell",
        "id": aas_id,
        "idShort": model,
        "assetInformation": {
            "assetKind": "Instance",
            "globalAssetId": make_urn("asset", model)
        },
        "submodels": [
            {
                "type": "ModelReference",
                "keys": [ { "type": "Submodel", "value": sm["id"] } ]
            }
            for sm in submodels
        ]
    }

def build_env(aas: Dict[str, Any], submodels: List[Dict[str, Any]]) -> Dict[str, Any]:
    return {
        "assetAdministrationShells": [aas],
        "submodels": submodels,
        "conceptDescriptions": []   # 语义库可后续补
    }

# ---------- 主流程 ----------
def stem_to_key(stem: str, model: str) -> str:
    """
    ur3_control_submodel -> control
    ur5_kinematics_submodel -> kinematics
    """
    s = stem.lower()
    s = re.sub(rf"^{re.escape(model.lower())}_", "", s)
    s = s.replace("_submodel", "")
    return s

def run():
    root = pathlib.Path(AAS_ROOT)
    if not root.exists():
        print("未找到 'aas/' 目录。")
        return

    for model_dir in sorted([p for p in root.iterdir() if p.is_dir()]):
        model = model_dir.name  # ur3 / ur5 / ...
        json_files = sorted(model_dir.glob("*.json"))
        if not json_files:
            continue

        submodels = []
        for jf in json_files:
            try:
                payload = read_json(jf)
            except Exception as e:
                print(f"[跳过] 读取失败 {jf}: {e}")
                continue

            key = stem_to_key(jf.stem, model)  # control / kinematics / ...
            sm  = build_submodel(model, key, payload)
            submodels.append(sm)

        if not submodels:
            continue

        aas = build_aas(model, submodels)
        env = build_env(aas, submodels)

        out_path = pathlib.Path(OUT_DIR) / f"{model}.env.json"
        write_json(out_path, env)
        print(f"[OK] 生成：{out_path}")

    print("\n完成。用 AASX Package Explorer：")
    print("  File → Import → Import AAS environment (JSON) ")
    print("  选择 out_env/<model>.env.json（如 out_env/ur3.env.json）导入。")
    print("  导入后可 Save as... 保存为 .aasx。")

if __name__ == "__main__":
    run()
