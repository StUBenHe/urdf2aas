#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
Build AAS v3 Environment JSONs by grouping submodels per robot folder.
Input: a folder like .../aas_v3_fixed/ur3/*.json (each a valid v3 Submodel)
Output: one Environment JSON per robot (e.g., env_v3_ur3.json) containing:
  - assetAdministrationShells[0] with idShort=<robot>, id=<urn:aas:aas:<robot>:x_1_0>
  - submodelRefs pointing to each submodel's id
  - submodels[] embedding the actual submodel objects
USAGE:
  python build_env_v3.py --root "D:/ur_description/aas_v3_fixed" --out "D:/ur_description/envs_v3"
"""
import json
import argparse
from pathlib import Path

def load_json(p: Path):
    with p.open("r", encoding="utf-8") as f:
        return json.load(f)

def build_env_for_robot(robot_dir: Path, out_dir: Path):
    # collect submodels (only *.json files)
    sms = []
    for p in sorted(robot_dir.glob("*.json")):
        try:
            obj = load_json(p)
            if isinstance(obj, dict) and obj.get("modelType") == "Submodel" and "id" in obj:
                sms.append(obj)
        except Exception:
            pass
    if not sms:
        return None

    robot = robot_dir.name
    aas = {
        "id": f"urn:aas:aas:{robot}:x_1_0",
        "idShort": robot,
        "submodelRefs": [
            {
                "type": "ModelReference",
                "keys": [
                    {"type": "Submodel", "value": sm["id"]}
                ]
            } for sm in sms
        ]
    }

    env = {
        "assetAdministrationShells": [aas],
        "submodels": sms,
        "conceptDescriptions": []
    }

    out_path = out_dir / f"env_v3_{robot}.json"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", encoding="utf-8") as f:
        json.dump(env, f, ensure_ascii=False, indent=2)
    return out_path

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True, help="Folder containing per-robot folders with fixed v3 Submodels (e.g., .../aas_v3_fixed)")
    ap.add_argument("--out", required=True, help="Output folder for environment JSONs")
    args = ap.parse_args()

    root = Path(args.root).resolve()
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)

    produced = []
    for robot_dir in sorted([p for p in root.iterdir() if p.is_dir()]):
        out_path = build_env_for_robot(robot_dir, out)
        if out_path:
            print(f"[OK] {robot_dir.name} -> {out_path}")
            produced.append(out_path)

    print(f"\nDone. Built {len(produced)} environment(s) into: {out}")
