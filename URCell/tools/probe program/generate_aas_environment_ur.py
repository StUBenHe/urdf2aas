#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
generate_aas_environment_ur.py

从 URDF 生成 submodel JSON（调用 generate_aas_submodels_ur.py），
再对 submodels 做 v3 语义补齐并生成 AAS Environment v3 JSON。

特性：
- 相对路径友好（以当前工作目录为基准）
- 可显式 --from-urdf，也可在缺少 submodels 时自动发现 URDF
- 不修改 File.value 路径（无 internal/external 模式）
- 默认 kind=Template
"""

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






# =========================
# CLI
# =========================
def main():
    ap = argparse.ArgumentParser(description="从 URDF 生成 submodels 并构建 AAS Environment v3（UR 系列）")
    ap.add_argument("target", help="单个机器人目录（如 types/submodel/ur3）或包含多个机器人的根目录（如 types/submodel）")
    ap.add_argument("--recursive", action="store_true", help="递归扫描 target 下所有子目录（每个含 JSON 的目录视为机器人目录）")
    ap.add_argument("--write", action="store_true", help="写回修改与环境文件（默认仅预览）")
    ap.add_argument("--force", action="store_true", help="强制覆盖已有 Submodel.semanticId")
    ap.add_argument("--force-idshort", action="store_true", help="覆盖 AAS.idShort")
    ap.add_argument("--kind", choices=["Type", "Template"], default="Template", help="Submodel.kind 与 AssetInformation.kind")
    ap.add_argument("--from-urdf", help="显式指定 URDF 路径（相对或绝对）；缺省时按常见路径自动发现")
    ap.add_argument("--regen", action="store_true", help="即使目录已有 JSON 也强制重新从 URDF 生成 submodels（会覆盖/追加具体取决于生成器实现）")
    args = ap.parse_args()

    target = Path(args.target)
    if not target.exists():
        print("❌ 路径不存在:", target)
        return

    urdf_hint = Path(args.from_urdf) if args.from_urdf else None
    # （注意：属性名是 from_urdf 不是 from-urdf）
    if args.from_urdf:
        urdf_hint = Path(args.from_urdf)

    if target.is_dir():
        if args.recursive:
            # 递归：凡是包含 .json 或空目录（需从 URDF 生成）的子目录都尝试处理
            robot_dirs: List[Path] = []
            for p in target.rglob("*"):
                if p.is_dir():
                    # 这里放宽：子目录即视为一个潜在机器人目录
                    robot_dirs.append(p)
        else:
            robot_dirs = [target]

        # 过滤：只处理最后一级目录名形如 ur* 或 igus_rebel 等
        # （简单规则：包含至少一个字母）
        robot_dirs = [d for d in robot_dirs if re.search(r'[A-Za-z]', d.name)]

        for robot_dir in sorted(robot_dirs):
            process_robot_dir(robot_dir, args.kind, args.force, args.force_idshort, args.write, urdf_hint, args.regen)
    else:
        print("❌ target 必须为目录。")

    print("\n🎯 所有任务完成。")


if __name__ == "__main__":
    main()
