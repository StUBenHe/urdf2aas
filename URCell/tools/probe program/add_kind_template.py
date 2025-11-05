import os, json

ROOT_DIR = r"C:\Users\benhe\Desktop\abschlussarbeit\repo-root\types\submodel"

for dirpath, dirnames, filenames in os.walk(ROOT_DIR):
    for filename in filenames:
        if filename.lower().endswith(".json"):
            path = os.path.join(dirpath, filename)
            try:
                with open(path, "r", encoding="utf-8-sig") as f:
                    data = json.load(f)
            except Exception as e:
                print(f"[跳过] 无法解析 {path} -> {e}")
                continue

            if not isinstance(data, dict):
                print(f"[跳过] 顶层不是对象 {path}")
                continue

            cur = data.get("kind", None)
            if cur is None or (isinstance(cur, str) and cur.strip().lower() in ("", "type")):
                data["kind"] = "Template"
                with open(path, "w", encoding="utf-8") as f:
                    json.dump(data, f, ensure_ascii=False, indent=2)
                print(f"[修改] {path}")
            else:
                print(f"[跳过] 已有 kind={cur} -> {path}")

print("处理完成。")
