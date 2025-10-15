
#!/usr/bin/env python3
"""
fix_aas_json.py

批量修正 AAS JSON 中 File 元素的 contentType，并可规范 File.value 的路径。
- 内嵌模式 (--mode internal)：保持/规范为包内路径（以 / 开头），如 /description/...
- 外链模式 (--mode external)：把相对路径改为 file:/// 绝对 URI（需要 --base 根目录）

用法：
  python fix_aas_json.py input.json output.json --mode internal
  python fix_aas_json.py input.json output.json --mode external --base "C:/path/to/repo-root/types"
可选：
  --dry-run      只统计不写出
  --verbose      打印被修改的条目
  --report       输出统计摘要
"""
import argparse, json, os, pathlib, urllib.parse
from typing import Any, Dict, Tuple

# 常见扩展名到 MIME 的映射
EXT2_MIME: Dict[str, str] = {
    ".stl":  "model/stl",
    ".dae":  "model/vnd.collada+xml",
    ".obj":  "model/obj",            # 非官方，但工具普遍接受
    ".glb":  "model/gltf-binary",
    ".gltf": "model/gltf+json",
    ".ply":  "application/octet-stream",
    ".step": "application/step",
    ".stp":  "application/step",
    ".iges": "application/iges",
    ".igs":  "application/iges",
    ".3ds":  "application/octet-stream",
    ".fbx":  "application/octet-stream",
    ".png":  "image/png",
    ".jpg":  "image/jpeg",
    ".jpeg": "image/jpeg",
    ".bmp":  "image/bmp",
    ".gif":  "image/gif",
    ".svg":  "image/svg+xml",
    ".mtl":  "text/plain",
    ".txt":  "text/plain",
}

def to_file_uri(p: pathlib.Path) -> str:
    # 生成 file:/// URI（会自动处理空格等编码）
    return p.resolve().as_uri()

def guess_ext_from_value(value: str) -> str:
    v = value or ""
    try:
        if v.lower().startswith("file:///"):
            v = urllib.parse.urlparse(v).path
        # Windows 驱动器在 URI 里是 /C:/...，这里直接取路径部分
        ext = os.path.splitext(v)[1].lower()
        return ext
    except Exception:
        return ""

def fix_content_type(value: str, old_ct: str) -> Tuple[str, bool]:
    """根据 value 的扩展名决定 MIME；若需要更新返回(新值, 是否修改)"""
    ext = guess_ext_from_value(value)
    target = EXT2_MIME.get(ext)
    changed = False
    if target:
        # 如果原来没有、或者原来是 text/*、application/octet-stream、或者不同，就更新
        if (not old_ct) or old_ct.lower().startswith("text/") or old_ct.lower() == "application/octet-stream" or (old_ct != target):
            changed = (old_ct != target)
            old_ct = target
    return old_ct or "", changed

def normalize_value(value: str, mode: str, base: pathlib.Path|None) -> Tuple[str, bool]:
    """按模式规范路径；返回(新值, 是否修改)"""
    if not isinstance(value, str):
        return value, False
    v = value.replace("\\", "/")
    # 已是绝对 file:// 的保持不变
    if v.lower().startswith("file:///"):
        return v, False

    # 识别几种常见相对写法
    # /description/... , ./description/... , description/...
    if v.startswith("/description/"):
        rel = v[1:]
    elif v.startswith("./description/"):
        rel = v[2:]
    elif v.startswith("description/"):
        rel = v
    else:
        rel = None

    if mode == "internal":
        if rel:
            nv = "/" + rel
            return (nv, nv != v)
        # 其他保持原样
        return v, False

    if mode == "external":
        if base is None:
            return v, False
        if rel:
            nv = to_file_uri(base.joinpath(rel))
            return nv, True
        # 不是以 / 开头也没有冒号的，按相对 base 处理
        if not (v.startswith("/") or ":" in v):
            nv = to_file_uri(base.joinpath(v))
            return nv, True
        return v, False

    return v, False

def walk(node: Any, mode: str, base: pathlib.Path|None, stats: Dict[str,int], verbose: bool) -> Any:
    if isinstance(node, dict):
        if node.get("modelType") == "File":
            value = node.get("value", "")
            ct = node.get("contentType", "")
            new_ct, ct_changed = fix_content_type(value, ct)
            if ct_changed:
                node["contentType"] = new_ct
                stats["mime_fixed"] += 1
                if verbose:
                    print(f"[MIME] {value} -> {new_ct}")

            new_value, v_changed = normalize_value(value, mode, base)
            if v_changed:
                node["value"] = new_value
                stats["value_fixed"] += 1
                if verbose:
                    print(f"[PATH] {value} -> {new_value}")

            stats["files_seen"] += 1

        # 递归子节点
        for k in list(node.keys()):
            node[k] = walk(node[k], mode, base, stats, verbose)

    elif isinstance(node, list):
        for i in range(len(node)):
            node[i] = walk(node[i], mode, base, stats, verbose)

    return node

def main():
    ap = argparse.ArgumentParser(description="Batch-fix AAS JSON: File.contentType + File.value 路径规范")
    ap.add_argument("input", help="输入 JSON 文件（从 AASX 导出）")
    ap.add_argument("output", help="输出 JSON 文件")
    ap.add_argument("--mode", choices=["internal", "external"], default="internal",
                    help="internal: 规范为包内路径（/description/...）；external: 改为 file:/// 绝对 URI（需 --base）")
    ap.add_argument("--base", help="external 模式下的根目录，如 C:/Users/me/repo-root/types")
    ap.add_argument("--dry-run", action="store_true", help="只统计不写文件")
    ap.add_argument("--verbose", action="store_true", help="打印修改详情")
    ap.add_argument("--report", action="store_true", help="输出统计摘要")
    args = ap.parse_args()

    base = pathlib.Path(args.base) if args.base else None
    stats = {"files_seen": 0, "mime_fixed": 0, "value_fixed": 0}

    with open(args.input, "r", encoding="utf-8") as f:
        data = json.load(f)

    fixed = walk(data, args.mode, base, stats, args.verbose)

    if args.report:
        print(f"Files seen: {stats['files_seen']}, MIME fixed: {stats['mime_fixed']}, Path fixed: {stats['value_fixed']}")

    if not args.dry_run:
        with open(args.output, "w", encoding="utf-8") as f:
            json.dump(fixed, f, ensure_ascii=False, indent=2)
        print("Wrote:", args.output)

if __name__ == "__main__":
    main()
