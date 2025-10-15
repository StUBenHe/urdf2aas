import argparse
from pathlib import Path
from xml.etree import ElementTree as ET
from datetime import datetime
import fnmatch
import os
import sys

def f2s(x, digits=6):
    if x is None:
        return "—"
    try:
        return f"{float(x):.{digits}g}"
    except Exception:
        return str(x)

# ---- Safe helpers ----
def get_child(el, tag):
    if el is None:
        return None
    child = el.find(tag)
    return child if child is not None else None

def get_child_attr(el, tag, attr):
    child = get_child(el, tag)
    if child is not None:
        return child.attrib.get(attr)
    return None

def get_child_text(el, tag):
    child = get_child(el, tag)
    if child is not None and child.text:
        return child.text.strip()
    return None

def parse_urdf(urdf_path: str):
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    robot_name = root.attrib.get("name", Path(urdf_path).stem)

    links = []
    joints = []
    transmissions = []
    materials = set()

    # global materials
    for m in root.findall("material"):
        n = m.attrib.get("name")
        if n:
            materials.add(n)

    # links
    for link in root.findall("link"):
        lname = link.attrib.get("name", "")

        inertial = get_child(link, "inertial")
        mass = None
        inertia = {}
        origin_xyz = None
        origin_rpy = None

        if inertial is not None:
            mass = get_child_attr(inertial, "mass", "value")
            inertia_tag = get_child(inertial, "inertia")
            if inertia_tag is not None:
                for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
                    if k in inertia_tag.attrib:
                        inertia[k] = inertia_tag.attrib.get(k)
            origin_tag = get_child(inertial, "origin")
            if origin_tag is not None:
                origin_xyz = origin_tag.attrib.get("xyz")
                origin_rpy = origin_tag.attrib.get("rpy")

        visual_meshes = []
        collision_meshes = []
        link_materials = []

        for visual in link.findall("visual"):
            geom = get_child(visual, "geometry")
            if geom is not None:
                mesh = get_child(geom, "mesh")
                if mesh is not None:
                    fn = mesh.attrib.get("filename")
                    if fn:
                        visual_meshes.append(fn)
            mat = get_child(visual, "material")
            if mat is not None:
                mname = mat.attrib.get("name")
                if mname:
                    link_materials.append(mname)

        for collision in link.findall("collision"):
            geom = get_child(collision, "geometry")
            if geom is not None:
                mesh = get_child(geom, "mesh")
                if mesh is not None:
                    fn = mesh.attrib.get("filename")
                    if fn:
                        collision_meshes.append(fn)

        links.append({
            "name": lname,
            "mass": mass,
            "inertia": inertia,
            "origin_xyz": origin_xyz,
            "origin_rpy": origin_rpy,
            "visual_meshes": visual_meshes,
            "collision_meshes": collision_meshes,
            "materials": link_materials
        })

    # joints
    for joint in root.findall("joint"):
        jname = joint.attrib.get("name", "")
        jtype = joint.attrib.get("type", "")

        parent = get_child_attr(joint, "parent", "link")
        child  = get_child_attr(joint, "child", "link")

        origin = get_child(joint, "origin")
        origin_xyz = origin.attrib.get("xyz") if origin is not None else None
        origin_rpy = origin.attrib.get("rpy") if origin is not None else None

        axis_tag = get_child(joint, "axis")
        axis = axis_tag.attrib.get("xyz") if axis_tag is not None else None

        limit = get_child(joint, "limit")
        limits = {}
        if limit is not None:
            for k in ("lower", "upper", "effort", "velocity"):
                if k in limit.attrib:
                    limits[k] = limit.attrib.get(k)

        joints.append({
            "name": jname,
            "type": jtype,
            "parent": parent,
            "child": child,
            "origin_xyz": origin_xyz,
            "origin_rpy": origin_rpy,
            "axis": axis,
            "limits": limits
        })

    # transmissions
    for trans in root.findall("transmission"):
        tname = trans.attrib.get("name", "")
        ttype = get_child_text(trans, "type")
        tjoints = []
        for j in trans.findall("joint"):
            jn = j.attrib.get("name")
            if not jn and j.text:
                jn = j.text.strip()
            if jn:
                tjoints.append(jn)
        tact = []
        for a in trans.findall("actuator"):
            an = a.attrib.get("name")
            if not an and a.text:
                an = a.text.strip()
            if an:
                tact.append(an)

        transmissions.append({
            "name": tname,
            "type": ttype,
            "joints": tjoints,
            "actuators": tact
        })

    return {
        "robot_name": robot_name,
        "links": links,
        "joints": joints,
        "transmissions": transmissions,
        "materials": sorted(materials),
    }

def build_markdown(d):
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    lines = []
    lines.append("# Submodel 描述文稿（URDF 自动生成）\n")
    lines.append(f"- 生成时间：{ts}")
    lines.append(f"- 机器人：**{d['robot_name']}**")
    lines.append(f"- 链接（links）：{len(d['links'])} 个")
    lines.append(f"- 关节（joints）：{len(d['joints'])} 个")
    lines.append(f"- 传动（transmissions）：{len(d['transmissions'])} 个")
    lines.append(f"- 材质（materials）：{len(d['materials'])} 种\n")
    lines.append("---\n")
    lines.append("## 建议的 AAS Submodels（草案）\n")

    # Structure
    lines.append("### 1) StructureSubmodel")
    for L in d["links"]:
        inertia_str = ", ".join([f"{k}={f2s(v)}" for k, v in L["inertia"].items()]) if L["inertia"] else ""
        lines.append(f"- Link `{L['name']}`: mass={f2s(L['mass'])}" + (f", inertia={{ {inertia_str} }}" if inertia_str else ""))
        if L["origin_xyz"] or L["origin_rpy"]:
            lines.append(f"  - 惯性原点：xyz={L['origin_xyz'] or '—'}, rpy={L['origin_rpy'] or '—'}")
    lines.append("")

    # Kinematics
    lines.append("### 2) KinematicsSubmodel")
    for J in d["joints"]:
        lines.append(f"- Joint `{J['name']}` ({J['type']}): {J['parent']} → {J['child']}, axis={J['axis'] or '—'}")
    lines.append("")

    # Dynamics
    lines.append("### 3) DynamicsSubmodel")
    for J in d["joints"]:
        if J["limits"]:
            lims = ", ".join([f"{k}={f2s(v)}" for k, v in J["limits"].items()])
            lines.append(f"- Joint `{J['name']}` limits: {{ {lims} }}")
    lines.append("")

    # Control
    lines.append("### 4) ControlSubmodel")
    if d["transmissions"]:
        for T in d["transmissions"]:
            lines.append(f"- Transmission `{T['name']}`: type={T['type'] or '—'}, joints={T['joints'] or '—'}, actuators={T['actuators'] or '—'}")
    else:
        lines.append("- （URDF 未提供 <transmission>）")
    lines.append("")

    # Visualization
    lines.append("### 5) VisualizationSubmodel")
    any_vis = False
    for L in d["links"]:
        parts = []
        if L["visual_meshes"]:
            parts.append("Visual: " + ", ".join(L["visual_meshes"]))
        if L["collision_meshes"]:
            parts.append("Collision: " + ", ".join(L["collision_meshes"]))
        if L["materials"]:
            parts.append("Materials: " + ", ".join(L["materials"]))
        if parts:
            any_vis = True
            lines.append(f"- Link `{L['name']}` → " + " | ".join(parts))
    if d["materials"]:
        lines.append(f"- 全局材质库：{', '.join(d['materials'])}")
    if not any_vis and not d["materials"]:
        lines.append("- （URDF 中未找到可视化/碰撞/材质信息）")
    lines.append("")

    # CD hints
    lines.append("## ConceptDescription（建议词条草案）")
    lines.append("- jointLowerLimit / jointUpperLimit / jointMaxVelocity / jointMaxEffort")
    lines.append("- linkMass / linkInertiaIxx/Iyy/Izz/Ixy/Ixz/Iyz")
    lines.append("- visualMeshUri / collisionMeshUri / materialName")
    lines.append("- transmissionType / actuatorName / transmissionJointRef\n")

    return "\n".join(lines)

def write_markdown_for_urdf(urdf_path: Path, out_root: Path):
    data = parse_urdf(str(urdf_path))
    ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    robot_dir = out_root / data['robot_name']
    robot_dir.mkdir(parents=True, exist_ok=True)
    out_path = robot_dir / f"submodel_draft_{data['robot_name']}_{ts}.md"
    md = build_markdown(data)
    out_path.write_text(md, encoding="utf-8")
    return data, out_path

def find_urdf_files(root: Path, pattern: str="*.urdf", recursive: bool=True):
    files = []
    if recursive:
        for dirpath, _, filenames in os.walk(root):
            for fn in filenames:
                if fnmatch.fnmatch(fn, pattern):
                    files.append(Path(dirpath) / fn)
    else:
        files.extend(root.glob(pattern))
    return sorted(files)

def main():
    ap = argparse.ArgumentParser(description="Generate Submodel narrative(s) from URDF file(s)")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--urdf", help="Path to a single URDF file")
    src.add_argument("--urdf-dir", help="Path to a directory containing URDF files")
    ap.add_argument("--pattern", default="*.urdf", help='Glob pattern for --urdf-dir (e.g., "*.urdf")')
    ap.add_argument("--recursive", action="store_true", help="Recurse into subdirectories when using --urdf-dir")
    ap.add_argument("--out", default=".", help="Output root directory")
    args = ap.parse_args()

    out_root = Path(args.out)
    out_root.mkdir(parents=True, exist_ok=True)
    index = []

    if args.urdf:
        urdf_path = Path(args.urdf)
        data, out_path = write_markdown_for_urdf(urdf_path, out_root)
        index.append((urdf_path, data, out_path))
    else:
        urdf_root = Path(args.urdf_dir)
        files = find_urdf_files(urdf_root, pattern=args.pattern, recursive=args.recursive)
        if not files:
            print("No URDF files found.", file=sys.stderr)
            sys.exit(1)
        for urdf_path in files:
            try:
                data, out_path = write_markdown_for_urdf(urdf_path, out_root)
                index.append((urdf_path, data, out_path))
            except Exception as e:
                print(f"[ERROR] {urdf_path}: {e}", file=sys.stderr)

    # Index
    idx_lines = ["# URDF → Submodel 描述稿索引", ""]
    idx_lines.append(f"- 生成时间：{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    idx_lines.append(f"- 输出根目录：`{out_root}`\n")
    for src_path, data, md_path in index:
        idx_lines.append(f"- **{data['robot_name']}** | links={len(data['links'])}, joints={len(data['joints'])}, transmissions={len(data['transmissions'])} → {md_path}")
    (out_root / "INDEX.md").write_text("\n".join(idx_lines), encoding="utf-8")

    for _, _, md_path in index:
        print(str(md_path))
    print(str(out_root / "INDEX.md"))

if __name__ == "__main__":
    main()

