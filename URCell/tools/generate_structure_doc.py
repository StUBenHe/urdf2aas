# File: C:\Users\benhe\Desktop\abschlussarbeit\URCell\tools\generate_structure_doc.py
import os

def generate_structure(root, max_depth=3, prefix="", extra_depth_paths=None):
    """Generate directory structure up to a given depth, with special deeper folders."""
    result = ""
    try:
        items = sorted([i for i in os.listdir(root) if not i.startswith('.')])
    except PermissionError:
        return result

    for i, item in enumerate(items):
        path = os.path.join(root, item)
        connector = "└─ " if i == len(items) - 1 else "├─ "
        result += f"{prefix}{connector}{item}\n"
        if os.path.isdir(path):
            # If this path is marked for extra depth, go one level deeper
            if extra_depth_paths and any(path.replace("\\", "/").endswith(p) for p in extra_depth_paths):
                depth = 4
            else:
                depth = max_depth
            if depth > 1:
                sub_prefix = prefix + ("    " if i == len(items) - 1 else "│   ")
                result += generate_structure(path, depth - 1, sub_prefix, extra_depth_paths)
    return result

def main():
    base_dir = r"C:\Users\benhe\Desktop\abschlussarbeit\URCell"
    output_path = os.path.join(base_dir, "urcell_structure.md")

    # Define the folders that need deeper exploration
    extra_depth_paths = [
        "types/submodel",
        "types/ur_description/urdf"
    ]

    print("📁 Generating directory structure (3 levels, deeper for submodel & urdf)...")
    structure = "URCell/\n" + generate_structure(base_dir, max_depth=3, extra_depth_paths=extra_depth_paths)

    with open(output_path, "w", encoding="utf-8") as f:
        f.write("# URCell File Structure (Custom Depth Overview)\n\n")
        f.write("```\n")
        f.write(structure)
        f.write("```\n")

    print(f"✅ File structure document generated at: {output_path}")

if __name__ == "__main__":
    main()

