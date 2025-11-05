#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
generate_aas_environment_all.py

自动遍历 URCell/types/submodel 下所有机器人目录，
为每个目录生成一个符合 AASX Explorer v3 可导入的环境 JSON：
  e.g. ur3_environment_v3.json, ur5_environment_v3.json

输出路径示例：
  types/submodel/ur3/ur3_environment_v3.json
  types/submodel/ur5/ur5_environment_v3.json
"""

import json
from pathlib import Path

# 🔹 导入你的三个工具模块
from kind_semantic_tool import process_json_file
from fix_aas_json import walk as fix_files

# ======================================================
# 全局路径配置
# ======================================================
ROOT = Path(__file__).resolve().parents[1] / "types" / "submodel"
print(f"🔍 Base path: {ROOT}")

# ======================================================
# 主处理逻辑
# ======================================================
def generate_aas_environment(robot_dir: Path):
    """为单个机器人目录生成 AAS 环境 JSON"""
    model_name = robot_dir.name
    submodels = []

    json_files = sorted(robot_dir.glob("*.json"))
    if not json_files:
        print(f"[跳过] {model_name}: 无 JSON 文件")
        return

    print(f"\n📦 处理机器人模型: {model_name}")

    # 读取并补齐每个 Submodel
    for f in json_files:
        try:
            data = json.loads(f.read_text(encoding="utf-8"))
            # 调用 kind_semantic_tool 自动补齐 kind / semanticId
            process_json_file(f, kind_value="Template", force=True, write=True)
            submodels.append(data)
        except Exception as e:
            print(f"[错误] {f.name}: {e}")

    if not submodels:
        print(f"[跳过] {model_name}: 无有效 Submodel 数据")
        return

    # ==================================================
    # 组装 AAS Environment 结构
    # ==================================================
    env = {
        "assetAdministrationShells": [{
            "id": f"urn:robot:aas:{model_name.upper()}_001",
            "idShort": f"{model_name.upper()}_Shell",
            "assetInformation": {
                "assetKind": "Instance",
                "globalAssetId": f"urn:robot:asset:{model_name.upper()}"
            },
            "submodels": [
                {
                    "keys": [
                        {
                            "type": "Submodel",
                            "value": sm.get("semanticId", {})
                                   .get("keys", [{}])[0]
                                   .get("value", f"urn:robot:submodel:{model_name}:{i}")
                        }
                    ]
                }
                for i, sm in enumerate(submodels)
            ]
        }],
        "submodels": submodels,
        "conceptDescriptions": []
    }

    # ==================================================
    # 调用 fix_aas_json 修复文件路径 / contentType
    # ==================================================
    try:
        fixed_env = fix_files(
            env,
            "internal",
            base=None,
            stats={"files_seen": 0, "mime_fixed": 0, "value_fixed": 0},
            verbose=False
        )
    except Exception as e:
        print(f"[警告] fix_aas_json 处理失败: {e}")
        fixed_env = env

    # ==================================================
    # 写出结果到当前机器人目录
    # ==================================================
    output_path = robot_dir / f"{model_name}_environment_v3.json"
    output_path.write_text(
        json.dumps(fixed_env, ensure_ascii=False, indent=2),
        encoding="utf-8"
    )

    print(f"✅ 已生成: {output_path}")


# ======================================================
# 主入口
# ======================================================
def main():
    # 遍历 submodel 下所有机器人子文件夹
    robot_dirs = [p for p in ROOT.iterdir() if p.is_dir()]
    if not robot_dirs:
        print("❌ 未发现任何机器人目录。请检查路径:", ROOT)
        return

    for robot_dir in sorted(robot_dirs):
        generate_aas_environment(robot_dir)

    print("\n🎯 所有机器人环境文件生成完成！")


if __name__ == "__main__":
    main()

