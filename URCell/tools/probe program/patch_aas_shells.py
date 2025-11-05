
#!/usr/bin/env python3
"""
patch_aas_shells.py

批量修改包含 "assetAdministrationShells" 的 AAS 环境 JSON：
- 将每个 AAS 的 assetInformation.kind 设为指定值（Template/Type/Instance）
- 若 idShort 为空或指定 --force-idshort，则自动推断并填写 AAS 的 idShort
  推断策略（优先级从高到低）：
    1) 根据 AAS 的 submodels 引用，找到第一个被引用的 Submodel 的 idShort，
       取其前缀（下划线前的部分），如 "UR3_StructureSafe" -> "UR3"
    2) 若找不到 idShort，则从该 Submodel 的 category 中提取形如 "Robot:UR3" 的型号前缀 -> "UR3"
    3) 若仍找不到，则从 AAS 的 id（IRI/URN）末段提取，如 ".../UR5e" -> "UR5e"
    4) 兜底：使用 "AAS_<index>"

只修改 AAS 顶层（不动子模型）。

用法：
  # 预览（不写文件）
  python patch_aas_shells.py env.json

  # 写回并设为 Template（AAS v3 常用）
  python patch_aas_shells.py env.json --write --kind Template

  # 目录递归处理
  python patch_aas_shells.py C:\repo-root\types --recursive --write --kind Template

  # 覆盖已有 idShort
  python patch_aas_shells.py env.json --write --force-idshort

"""

import argparse, json, re
from pathlib import Path
from typing import Any, Dict, List, Optional

SM_BY_ID_CACHE: Dict[Path, Dict[str, Dict[str, Any]]] = {}

def build_sm_index(root: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
    idx: Dict[str, Dict[str, Any]] = {}
    sms = root.get("submodels")
    if isinstance(sms, list):
        for sm in sms:
            if isinstance(sm, dict) and sm.get("modelType") == "Submodel":
                sid = sm.get("id")
                if isinstance(sid, str):
                    idx[sid] = sm
    return idx

def get_first_referenced_submodel_id(smref_obj: Dict[str, Any]) -> Optional[str]:
    # AAS v2/3 通常是 {"type": "ModelReference"/"Reference", "keys":[{"type":"Submodel","value":"..."}, ...]}
    keys = smref_obj.get("keys")
    if not isinstance(keys, list):
        return None
    # 取最后一个 type==Submodel 的 key 的 value
    for k in reversed(keys):
        if isinstance(k, dict) and str(k.get("type","")).lower() == "submodel":
            val = k.get("value")
            if isinstance(val, str) and val:
                return val
    return None

def infer_prefix_from_submodel(sm: Dict[str, Any]) -> Optional[str]:
    # 1) 从 submodel.idShort 取前缀（UR3_xxx -> UR3）
    ids = sm.get("idShort")
    if isinstance(ids, str) and ids:
        pref = ids.split("_", 1)[0]
        if pref:
            return pref
    # 2) 从 submodel.category 提取 Robot:UR3
    cat = sm.get("category")
    if isinstance(cat, str):
        m = re.search(r'UR\d+\w?', cat, flags=re.I)
        if m:
            return m.group(0)
    return None

def infer_prefix_from_aas_id(aas: Dict[str, Any]) -> Optional[str]:
    aid = aas.get("id")
    if not isinstance(aid, str):
        return None
    # 取最后一个 / 或 : 后的片段，找 UR 型号
    tail = re.split(r'[/:]', aid)[-1]
    m = re.search(r'UR\d+\w?', tail, flags=re.I)
    if m:
        return m.group(0)
    return None

def normalize_idshort(s: str) -> str:
    # 清理非法字符，仅保留字母数字下划线，且不能以非字母开头
    s2 = re.sub(r'[^A-Za-z0-9_]', '_', s).strip('_')
    if not s2 or not re.match(r'[A-Za-z]', s2[0]):
        s2 = "AAS_" + s2
    return s2

def process_env_json(path: Path, kind_value: str, force_idshort: bool, write: bool) -> (int, int):
    data = json.loads(path.read_text(encoding="utf-8"))
    shells = data.get("assetAdministrationShells")
    if not isinstance(shells, list) or not shells:
        print(f"SKIP(no shells): {path}")
        return 0, 0

    # 建立 submodel 索引，以便根据 AAS 的 submodels 引用去找对应的 Submodel.idShort/category
    if path not in SM_BY_ID_CACHE:
        SM_BY_ID_CACHE[path] = build_sm_index(data)
    sm_index = SM_BY_ID_CACHE[path]

    seen = upd = 0
    for i, aas in enumerate(shells):
        if not isinstance(aas, dict):
            continue
        changed = False

        # 1) kind
        ai = aas.get("assetInformation")
        if not isinstance(ai, dict):
            ai = {}
            aas["assetInformation"] = ai
            changed = True
        if ai.get("kind") != kind_value:
            ai["kind"] = kind_value
            changed = True
        # 同步写 assetKind（兼容某些版本/映射）
        if ai.get("assetKind") != kind_value:
            ai["assetKind"] = kind_value
            changed = True

        # 2) idShort
        cur = aas.get("idShort")
        if force_idshort or not isinstance(cur, str) or not cur.strip():
            # 优先按 submodels 引用找
            pref = None
            refs = aas.get("submodels")  # AAS 里是对 Submodel 的引用数组
            if isinstance(refs, list) and sm_index:
                for ref in refs:
                    if isinstance(ref, dict):
                        sid = get_first_referenced_submodel_id(ref)
                        if sid and sid in sm_index:
                            pref = infer_prefix_from_submodel(sm_index[sid])
                            if pref:
                                break
            # 其次尝试从 AAS 自己的 id 里找
            if not pref:
                pref = infer_prefix_from_aas_id(aas)
            # 兜底
            if not pref:
                pref = f"AAS_{i+1}"
            new_ids = normalize_idshort(pref)
            if new_ids != cur:
                aas["idShort"] = new_ids
                changed = True

        if changed:
            upd += 1
        seen += 1

    if write and upd > 0:
        path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"{'WROTE' if write else 'PREVIEW'}: {path}  seen={seen} updated={upd}")
    return seen, upd

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("target", help="AAS 环境 JSON 文件或目录（需包含 assetAdministrationShells）")
    ap.add_argument("--recursive", action="store_true", help="目标为目录时递归处理 *.json")
    ap.add_argument("--write", action="store_true", help="实际写回，否则仅预览")
    ap.add_argument("--kind", choices=["Template", "Type", "Instance"], default="Template", help="AssetInformation.kind 目标值")
    ap.add_argument("--force-idshort", action="store_true", help="即使已有 idShort 也覆盖")
    args = ap.parse_args()

    p = Path(args.target)
    paths = []
    if p.is_dir():
        paths = list(p.rglob("*.json")) if args.recursive else list(p.glob("*.json"))
    else:
        paths = [p]

    total_seen = total_upd = 0
    for fp in paths:
        try:
            s, u = process_env_json(fp, args.kind, args.force_idshort, args.write)
            total_seen += s; total_upd += u
        except Exception as e:
            print(f"ERROR {fp}: {e}")

    print(f"Done. seen={total_seen}, updated={total_upd}, mode={'write' if args.write else 'preview'}")

if __name__ == "__main__":
    main()
