
#!/usr/bin/env python3
"""
set_kind_and_semantic.py

批量为 AAS Submodel JSON 设置：
- kind（默认 Type；v3 可传 --kind Template）
- semanticId（按文件名/目录推断类别：Structure/Kinematics/Dynamics/Control/Visualization/Safety/Environment）

兼容常见的 AASX JSON 结构（v2.0.1 / v3 风格）。
"""

import argparse, json, re
from pathlib import Path
from typing import Dict, Any

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

def infer_category(p: Path):
    text = (p.parent.name + "_" + p.stem).lower()
    m = PAT.search(text)
    return m.group(0).lower() if m else None

def make_semantic(urn: str) -> Dict[str, Any]:
    return {
        "type": "ExternalReference",
        "keys": [
            {"type": "GlobalReference", "value": urn}
        ]
    }

def process_json(path: Path, kind_value: str, force: bool, write: bool) -> bool:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or data.get("modelType") != "Submodel":
        return False

    changed = False

    # kind
    if data.get("kind") != kind_value:
        data["kind"] = kind_value
        changed = True

    # semanticId
    sem = data.get("semanticId")
    need_set = force or (not isinstance(sem, dict) or not sem.get("keys") or not sem.get("keys")[0].get("value"))
    if need_set:
        cat = infer_category(path)
        urn = URN_MAP.get(cat, f"urn:robot:submodel:{path.stem}:1:0")
        data["semanticId"] = make_semantic(urn)
        changed = True

    if changed and write:
        path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")

    print(f"{'UPDATED' if changed else 'SKIPPED'}: {path.name}  kind={data.get('kind')}  semanticId={data.get('semanticId',{})}")
    return changed

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("target", help="单个 JSON 文件或包含 JSON 的目录")
    ap.add_argument("--recursive", action="store_true", help="递归处理目录")
    ap.add_argument("--write", action="store_true", help="实际写回（不加则只预览）")
    ap.add_argument("--force", action="store_true", help="即使已有 semanticId 也覆盖")
    ap.add_argument("--kind", choices=["Type", "Template"], default="Type", help="Submodel.kind 值（v3 推荐 Template）")
    args = ap.parse_args()

    target = Path(args.target)
    paths = []
    if target.is_dir():
        paths = list(target.rglob("*.json")) if args.recursive else list(target.glob("*.json"))
    else:
        paths = [target]

    total = upd = 0
    for p in paths:
        try:
            if process_json(p, args.kind, args.force, args.write):
                upd += 1
            total += 1
        except Exception as e:
            print(f"ERROR {p}: {e}")

    print(f"Done. seen={total}, updated={upd}, mode={'write' if args.write else 'preview'}")

if __name__ == "__main__":
    main()
