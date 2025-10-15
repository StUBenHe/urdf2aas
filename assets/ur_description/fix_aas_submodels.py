#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Batch-fix AAS v3 Submodel JSON files (strict idShort + Instance).
- Enforces kind="Instance"
- Sanitizes ALL idShorts: [A-Za-z][A-Za-z0-9_]*
- Rewrites bare elements into SMC->Property[] when needed
- Normalizes valueType to xs:* and stringifies values
Usage:
  python fix_aas_submodels_strict.py --root "D:/ur_description/aas" --out "D:/ur_description/aas_v3_fixed"
"""
import json
import argparse
import re
from pathlib import Path

DEFAULT_KIND = "Instance"   # per your requirement

SEMANTIC_MAP = {
    "control": "urn:sem:robot:control",
    "dynamics": "urn:sem:robot:dynamics",
    "kinematics": "urn:sem:robot:kinematics",
    "safety": "urn:sem:robot:safety",
    "structure": "urn:sem:robot:structure",
    "visualization": "urn:sem:robot:visualization",
}

def sanitize_idshort(s: str) -> str:
    if not isinstance(s, str) or not s:
        s = "sm"
    s = s.replace('-', '_')
    s = ''.join(ch if (ch.isalpha() or ch.isdigit() or ch == '_') else '_' for ch in s)
    while '__' in s:
        s = s.replace('__', '_')
    if not s[0].isalpha():
        s = 'sm' + s
    return s

def guess_semantic_id(fname: str):
    lower = fname.lower()
    for k, urn in SEMANTIC_MAP.items():
        if k in lower:
            return urn
    return None

def xs_value_type(val):
    if isinstance(val, bool):
        return "xs:boolean", "true" if val else "false"
    if isinstance(val, int):
        return "xs:int", str(val)
    if isinstance(val, float):
        return "xs:double", str(val)
    return "xs:string", "" if val is None else str(val)

def to_property(name, val):
    vt, sval = xs_value_type(val)
    return {"modelType":"Property","idShort":sanitize_idshort(name),"valueType":vt,"value":sval}

def normalize_valueType_inplace(obj):
    if isinstance(obj, dict):
        if "valueType" in obj:
            vt = obj["valueType"]
            if isinstance(vt, str):
                lut = {
                    "string":"xs:string","String":"xs:string",
                    "double":"xs:double","float":"xs:double","Double":"xs:double","Float":"xs:double",
                    "int":"xs:int","integer":"xs:int","long":"xs:int",
                    "boolean":"xs:boolean","Boolean":"xs:boolean",
                }
                if vt.startswith("xs:"):
                    pass
                elif vt in lut:
                    obj["valueType"] = lut[vt]
                else:
                    obj["valueType"] = "xs:string"
        if "idShort" in obj:
            obj["idShort"] = sanitize_idshort(obj["idShort"])
        for v in obj.values():
            normalize_valueType_inplace(v)
    elif isinstance(obj, list):
        for v in obj:
            normalize_valueType_inplace(v)

def stringify_property_values(obj):
    if isinstance(obj, dict):
        if obj.get("modelType") == "Property" and "value" in obj and not isinstance(obj["value"], str):
            obj["value"] = xs_value_type(obj["value"])[1]
        for v in obj.values():
            stringify_property_values(v)
    elif isinstance(obj, list):
        for v in obj:
            stringify_property_values(v)

def as_collection_with_properties(node_idshort: str, val_obj: dict):
    props = []
    keys = ["lower","upper","effort","velocity"]
    for k in keys:
        props.append(to_property(k, None if val_obj is None else val_obj.get(k)))
    return {"modelType":"SubmodelElementCollection","idShort":sanitize_idshort(node_idshort),"value":props}

def convert_file(src_path: Path, dst_root: Path, root: Path):
    with src_path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    family = src_path.parent.name
    name = src_path.stem
    sm_idshort = re.sub(r"submodel$", "", name, flags=re.IGNORECASE).replace("_", "-")
    sm_idshort = sanitize_idshort(sm_idshort)

    module = None
    for key in SEMANTIC_MAP:
        if key in name.lower():
            module = key
            break
    if module is None:
        module = "custom"

    urn_id = f"urn:aas:submodel:{family}:{module}:x_1_0"

    if isinstance(data, dict) and data.get("modelType") == "Submodel":
        data.setdefault("id", urn_id)
        data["idShort"] = sanitize_idshort(data.get("idShort", sm_idshort))
        data["kind"] = DEFAULT_KIND
        normalize_valueType_inplace(data)
        stringify_property_values(data)
        dst_path = dst_root / src_path.relative_to(root)
        dst_path.parent.mkdir(parents=True, exist_ok=True)
        with dst_path.open("w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        return True, "normalized-existing-submodel", str(dst_path)

    if isinstance(data, dict) and "submodelElements" in data:
        elements = []
        for el in data["submodelElements"]:
            node_id = sanitize_idshort(el.get("idShort", "unnamed"))
            val = el.get("value")
            if isinstance(val, dict) or val is None:
                elements.append(as_collection_with_properties(node_id, val))
            elif isinstance(val, list):
                normalize_valueType_inplace(val)
                elements.append({"modelType":"SubmodelElementCollection","idShort":node_id,"value":val})
            else:
                elements.append(to_property(node_id, val))

        submodel = {
            "modelType": "Submodel",
            "id": urn_id,
            "idShort": sm_idshort,
            "kind": DEFAULT_KIND,
            "submodelElements": elements
        }

        sem = guess_semantic_id(name)
        if sem:
            submodel["semanticId"] = {"type":"ModelReference","keys":[{"type":"GlobalReference","value":sem}]}

        normalize_valueType_inplace(submodel)
        stringify_property_values(submodel)

        dst_path = dst_root / src_path.relative_to(root)
        dst_path.parent.mkdir(parents=True, exist_ok=True)
        with dst_path.open("w", encoding="utf-8") as f:
            json.dump(submodel, f, ensure_ascii=False, indent=2)
        return True, "wrapped-into-submodel", str(dst_path)

    return False, "unrecognized-structure", str(src_path)

def scan_files(root: Path):
    return list(root.rglob("*_submodel.json"))

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True, help="Folder where your 'aas' lives (e.g., D:/ur_description/aas)")
    ap.add_argument("--out", required=True, help="Output folder for fixed files")
    args = ap.parse_args()

    root = Path(args.root).resolve()
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)

    files = scan_files(root)
    total = len(files); ok = 0
    for src in files:
        success, mode, dst = convert_file(src, out, root)
        print(f"[{'OK' if success else 'SKIP'}] {mode:<26} {src} -> {dst}")
        if success: ok += 1

    print(f"\nDone. {ok}/{total} files converted into: {out}")
