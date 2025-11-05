
#!/usr/bin/env python3
"""
patch_env_json.py

为两种常见 JSON 结构批量补齐 Submodel.kind 与 semanticId：
1) 单个子模型 JSON：根对象是 {"modelType": "Submodel", ...}
2) AAS 环境/聚合 JSON：根对象含有 "submodels": [ {...Submodel...}, ... ]

用法：
  # 处理单个文件（预览）
  python patch_env_json.py path\to\file.json

  # 真正写回 + kind=Template
  python patch_env_json.py path\to\file.json --write --kind Template

  # 递归处理目录
  python patch_env_json.py C:\repo\types\submodel --recursive --write --kind Template

选项：
  --force    已有 semanticId 也覆盖
  --kind     Type 或 Template（默认 Type；AAS v3 建议 Template）
  --recursive  目标为目录时递归
"""
import argparse, json, re
from pathlib import Path
from typing import Dict, Any, Tuple

URN_MAP = {
    "structure":       "urn:robot:submodel:Structure:1:0",
    "kinematics":      "urn:robot:submodel:Kinematics:1:0",
    "dynamics":        "urn:robot:submodel:Dynamics:1:0",
    "control":         "urn:robot:submodel:Control:1:0",
    "visualization":   "urn:robot:submodel:Visualization:1:0",
    "safety":          "urn:robot:submodel:Safety:1:0",
    "environment":     "urn:robot:submodel:Environment:1:0",
}

PAT = re.compile("|".join(sorted(map(re.escape, URN_MAP.keys()), key=len, reverse=True)), re.I)

def infer_category_from_text(text: str) -> str | None:
    if not text:
        return None
    m = PAT.search(text.lower())
    return m.group(0) if m else None

def infer_category(path: Path, submodel_obj: Dict[str, Any]) -> str | None:
    text = (path.parent.name + "_" + path.stem + "_" + str(submodel_obj.get("idShort","")) + "_" + str(submodel_obj.get("id",""))).lower()
    return infer_category_from_text(text)

def make_semantic(urn: str) -> Dict[str, Any]:
    return {
        "type": "ExternalReference",
        "keys": [
            {"type": "GlobalReference", "value": urn}
        ]
    }

def patch_one_sm(sm: Dict[str, Any], kind_value: str, force: bool, path_hint: Path) -> Tuple[bool, Dict[str, Any]]:
    changed = False
    if sm.get("modelType") != "Submodel":
        return False, sm

    # kind
    if sm.get("kind") != kind_value:
        sm["kind"] = kind_value
        changed = True

    # semanticId
    sem = sm.get("semanticId")
    need_set = force or (not isinstance(sem, dict) or not sem.get("keys") or not sem.get("keys")[0].get("value"))
    if need_set:
        cat = infer_category(path_hint, sm)
        urn = URN_MAP.get(cat or "", f"urn:robot:submodel:{sm.get('idShort') or path_hint.stem}:1:0")
        sm["semanticId"] = make_semantic(urn)
        changed = True

    return changed, sm

def process_json(path: Path, kind_value: str, force: bool, write: bool) -> Tuple[int,int]:
    data = json.loads(path.read_text(encoding="utf-8"))
    seen = updated = 0

    def handle_sm_obj(obj: Dict[str, Any]):
        nonlocal seen, updated
        ok, newobj = patch_one_sm(obj, kind_value, force, path)
        if ok:
            updated += 1
        seen += 1

    if isinstance(data, dict) and data.get("modelType") == "Submodel":
        handle_sm_obj(data)
    elif isinstance(data, dict) and isinstance(data.get("submodels"), list):
        for sm in data["submodels"]:
            if isinstance(sm, dict) and sm.get("modelType") == "Submodel":
                handle_sm_obj(sm)
    else:
        # 其它形态不处理
        pass

    if write and updated > 0:
        path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")

    print(f"{'WROTE' if write else 'PREVIEW'}: {path}  seen={seen} updated={updated}")
    return seen, updated

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("target", help="文件或目录")
    ap.add_argument("--recursive", action="store_true", help="目录下递归处理 *.json")
    ap.add_argument("--write", action="store_true", help="实际写回")
    ap.add_argument("--force", action="store_true", help="覆盖已有 semanticId")
    ap.add_argument("--kind", choices=["Type","Template"], default="Type", help="Submodel.kind 目标值")
    args = ap.parse_args()

    target = Path(args.target)
    paths = []
    if target.is_dir():
        paths = list(target.rglob("*.json")) if args.recursive else list(target.glob("*.json"))
    else:
        paths = [target]

    total_seen = total_upd = 0
    for p in paths:
        try:
            seen, upd = process_json(p, args.kind, args.force, args.write)
            total_seen += seen
            total_upd += upd
        except Exception as e:
            print(f"ERROR {p}: {e}")

    print(f"Done. total_seen={total_seen}, total_updated={total_upd}, mode={'write' if args.write else 'preview'}")

if __name__ == "__main__":
    main()
