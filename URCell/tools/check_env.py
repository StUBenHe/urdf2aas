#!/usr/bin/env python3
import sys
import argparse
import importlib
import platform
import json
from datetime import datetime
from pathlib import Path
from typing import List, Tuple

DEFAULT_PACKAGES = [
    "urdfpy", "networkx", "numpy", "scipy", "pandas", "numba",
    "PyYAML", "trimesh", "pyrender", "torch", "transformers"
]

def check_packages(packages: List[str]) -> Tuple[list, list]:
    ok, missing = [], []
    for pkg in packages:
        try:
            mod = importlib.import_module(pkg)
            ver = getattr(mod, "__version__", "N/A")
            ok.append((pkg, ver))
        except Exception as e:
            missing.append((pkg, str(e)))
    return ok, missing

def write_txt(path: Path, ok: list, missing: list) -> None:
    lines = []
    lines.append("===================================================")
    lines.append("🔍 AAS & URCell Environment Check Utility")
    lines.append(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append("===================================================")
    lines.append(f"🐍 Python Version: {sys.version.split()[0]}")
    lines.append(f"💻 Platform: {platform.system()} {platform.release()}")
    lines.append("---------------------------------------------------")
    for name, ver in ok:
        lines.append(f"✅ {name:15} v{ver}")
    for name, err in missing:
        lines.append(f"❌ {name:15} NOT FOUND ({err})")
    lines.append("---------------------------------------------------")
    lines.append("✅ Environment check completed." if not missing else "⚠️ Environment check found missing packages.")
    path.write_text("\n".join(lines), encoding="utf-8")

def write_json(path: Path, ok: list, missing: list) -> None:
    payload = {
        "generated": datetime.now().isoformat(timespec="seconds"),
        "python": sys.version.split()[0],
        "platform": {"system": platform.system(), "release": platform.release()},
        "packages": [{"name": n, "version": v, "status": "ok"} for n, v in ok] +                         [{"name": n, "error": e, "status": "missing"} for n, e in missing]
    }
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")

def main():
    parser = argparse.ArgumentParser(description="Check AAS/URCell Python environment and write a report.")
    parser.add_argument("--output", "-o", default="env_report.txt", help="Output report path (txt). Default: env_report.txt (next to this script)")
    parser.add_argument("--json", action="store_true", help="Also write env_report.json alongside the txt report")
    parser.add_argument("--packages", "-p", nargs="*", default=DEFAULT_PACKAGES, help="Override package list to check")
    args = parser.parse_args()

    # Resolve output path
    out_path = Path(args.output)
    if not out_path.is_absolute():
        # default to script directory if relative
        out_path = Path(__file__).parent / out_path

    ok, missing = check_packages(args.packages)

    # Write reports
    write_txt(out_path, ok, missing)
    if args.json:
        write_json(out_path.with_suffix(".json"), ok, missing)

    # Console echo
    print((out_path).read_text(encoding="utf-8"))
    if args.json:
        print(f"Also wrote JSON: {out_path.with_suffix('.json')}")

    # Exit code: 0 if all good, 1 if anything missing
    sys.exit(0 if not missing else 1)

if __name__ == "__main__":
    main()
