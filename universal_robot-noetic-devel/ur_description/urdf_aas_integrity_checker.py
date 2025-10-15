
import os
import json
import argparse
import csv
import re
from pathlib import Path
from xml.etree import ElementTree as ET
from collections import defaultdict
from datetime import datetime

def parse_float(val):
    try:
        return float(val)
    except Exception:
        return None

def parse_urdf(urdf_path: str):
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    ns = ""  # URDF generally has no XML namespace

    robot_name = root.attrib.get("name", "unknown_robot")

    links = {}
    for link in root.findall(f"{ns}link"):
        lname = link.attrib.get("name")
        if not lname:
            continue
        link_info = {"name": lname, "inertial": {}, "visual_meshes": [], "collision_meshes": [], "materials": []}

        inertial = link.find(f"{ns}inertial")
        if inertial is not None:
            mass = inertial.find(f"{ns}mass")
            inertia = inertial.find(f"{ns}inertia")
            origin = inertial.find(f"{ns}origin")
            link_info["inertial"]["mass"] = parse_float(mass.attrib.get("value")) if mass is not None else None
            if inertia is not None:
                link_info["inertial"]["ixx"] = parse_float(inertia.attrib.get("ixx"))
                link_info["inertial"]["ixy"] = parse_float(inertia.attrib.get("ixy"))
                link_info["inertial"]["ixz"] = parse_float(inertia.attrib.get("ixz"))
                link_info["inertial"]["iyy"] = parse_float(inertia.attrib.get("iyy"))
                link_info["inertial"]["iyz"] = parse_float(inertia.attrib.get("iyz"))
                link_info["inertial"]["izz"] = parse_float(inertia.attrib.get("izz"))
            if origin is not None:
                link_info["inertial"]["origin_xyz"] = origin.attrib.get("xyz")
                link_info["inertial"]["origin_rpy"] = origin.attrib.get("rpy")

        for visual in link.findall(f"{ns}visual"):
            geom = visual.find(f"{ns}geometry")
            if geom is not None:
                mesh = geom.find(f"{ns}mesh")
                if mesh is not None:
                    filename = mesh.attrib.get("filename")
                    if filename:
                        link_info["visual_meshes"].append(filename)
            material = visual.find(f"{ns}material")
            if material is not None:
                mname = material.attrib.get("name")
                if mname:
                    link_info["materials"].append(mname)

        for collision in link.findall(f"{ns}collision"):
            geom = collision.find(f"{ns}geometry")
            if geom is not None:
                mesh = geom.find(f"{ns}mesh")
                if mesh is not None:
                    filename = mesh.attrib.get("filename")
                    if filename:
                        link_info["collision_meshes"].append(filename)

        links[lname] = link_info

    joints = {}
    for joint in root.findall(f"{ns}joint"):
        jname = joint.attrib.get("name")
        jtype = joint.attrib.get("type")
        if not jname:
            continue
        jinfo = {"name": jname, "type": jtype, "parent": None, "child": None, "origin": {}, "axis": None, "limit": {}}
        parent = joint.find(f"{ns}parent")
        child = joint.find(f"{ns}child")
        origin = joint.find(f"{ns}origin")
        axis = joint.find(f"{ns}axis")
        limit = joint.find(f"{ns}limit")

        if parent is not None:
            jinfo["parent"] = parent.attrib.get("link")
        if child is not None:
            jinfo["child"] = child.attrib.get("link")
        if origin is not None:
            jinfo["origin"]["xyz"] = origin.attrib.get("xyz")
            jinfo["origin"]["rpy"] = origin.attrib.get("rpy")
        if axis is not None:
            jinfo["axis"] = axis.attrib.get("xyz")
        if limit is not None:
            for key in ("lower", "upper", "effort", "velocity"):
                if key in limit.attrib:
                    jinfo["limit"][key] = parse_float(limit.attrib.get(key))

        joints[jname] = jinfo

    # Parse transmissions if present (common in ROS URDFs)
    transmissions = {}
    for trans in root.findall(f"{ns}transmission"):
        tname = trans.attrib.get("name")
        tinfo = {"name": tname, "type": None, "joints": [], "actuators": []}
        ttype = trans.find(f"{ns}type")
        if ttype is not None and ttype.text:
            tinfo["type"] = ttype.text.strip()
        for j in trans.findall(f"{ns}joint"):
            jn = j.attrib.get("name") or j.text
            if jn:
                tinfo["joints"].append(jn)
        for a in trans.findall(f"{ns}actuator"):
            an = a.attrib.get("name") or a.text
            if an:
                tinfo["actuators"].append(an)
        if tname:
            transmissions[tname] = tinfo

    materials = set()
    for m in root.findall(f"{ns}material"):
        mname = m.attrib.get("name")
        if mname:
            materials.add(mname)

    return {
        "robot_name": robot_name,
        "links": links,
        "joints": joints,
        "transmissions": transmissions,
        "materials": sorted(list(materials)),
    }

def walk_json(obj, path=""):
    """Yield (path, key, value, container) for all leaf values and dicts that have 'name' or 'idShort'."""
    if isinstance(obj, dict):
        # index potential identifiers
        ident_keys = ["name", "idShort", "id", "semanticId", "displayName"]
        found_ident = {k: obj.get(k) for k in ident_keys if k in obj}
        if found_ident:
            yield (path, "__ident__", found_ident, obj)
        for k, v in obj.items():
            new_path = f"{path}.{k}" if path else k
            yield from walk_json(v, new_path)
    elif isinstance(obj, list):
        for idx, v in enumerate(obj):
            new_path = f"{path}[{idx}]"
            yield from walk_json(v, new_path)
    else:
        yield (path, None, obj, None)

def load_json_dir(json_dir: str):
    index = {
        "files": [],
        "idents": [],  # list of (file, path, ident_dict)
        "text": defaultdict(list),  # value string -> list of (file, path)
    }
    for root, _, files in os.walk(json_dir):
        for fname in files:
            if not fname.lower().endswith(".json"):
                continue
            fpath = os.path.join(root, fname)
            try:
                with open(fpath, "r", encoding="utf-8") as f:
                    data = json.load(f)
            except Exception as e:
                # Skip invalid JSON but record
                index["files"].append({"file": fpath, "error": str(e)})
                continue
            index["files"].append({"file": fpath, "error": None})
            for (path, key, val, container) in walk_json(data):
                if key == "__ident__":
                    index["idents"].append({"file": fpath, "path": path, "ident": val, "container": container})
                if isinstance(val, str):
                    sval = val.strip()
                    if sval:
                        index["text"][sval].append((fpath, path))
    return index

def find_candidates_by_name(index, name: str):
    # Direct matches in idents (name or idShort equals name)
    hits = []
    for rec in index["idents"]:
        ident = rec["ident"]
        for k in ("name", "idShort", "displayName"):
            if ident.get(k) == name:
                hits.append(rec)
                break
    # Text matches anywhere
    text_hits = index["text"].get(name, [])
    return hits, text_hits

def compare_joint_limits(urdf_joint, candidate_container):
    """Attempt to compare limits if candidate_container has similar fields."""
    # Heuristic: look for fields lower/upper/effort/velocity nested anywhere inside the container
    found = {}
    def scan(d):
        if isinstance(d, dict):
            for k, v in d.items():
                lk = k.lower()
                if lk in ("lower", "upper", "effort", "velocity", "max_velocity", "max_effort"):
                    if isinstance(v, (int, float, str)):
                        try:
                            found[lk] = float(v)
                        except Exception:
                            pass
                else:
                    scan(v)
        elif isinstance(d, list):
            for x in d:
                scan(x)
    scan(candidate_container)
    mismatches = []
    matches = []
    for key in ("lower", "upper", "effort", "velocity"):
        uval = urdf_joint.get("limit", {}).get(key)
        # alias support
        cand_key = key
        if key == "velocity" and "max_velocity" in found and "velocity" not in found:
            cand_key = "max_velocity"
        if key == "effort" and "max_effort" in found and "effort" not in found:
            cand_key = "max_effort"
        cval = found.get(cand_key)
        if uval is None or cval is None:
            continue
        # compare with tolerance
        tol = 1e-6
        if abs(uval - cval) <= tol:
            matches.append((key, uval, cval))
        else:
            mismatches.append((key, uval, cval))
    return matches, mismatches

def compare_link_inertial(urdf_link, candidate_container):
    found = {}
    def scan(d):
        if isinstance(d, dict):
            for k, v in d.items():
                lk = k.lower()
                if lk in ("mass", "ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
                    if isinstance(v, (int, float, str)):
                        try:
                            found[lk] = float(v)
                        except Exception:
                            pass
                else:
                    scan(v)
        elif isinstance(d, list):
            for x in d:
                scan(x)
    scan(candidate_container)
    mismatches = []
    matches = []
    for key in ("mass", "ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
        uval = urdf_link.get("inertial", {}).get(key)
        cval = found.get(key)
        if uval is None or cval is None:
            continue
        tol = 1e-6
        if abs(uval - cval) <= tol:
            matches.append((key, uval, cval))
        else:
            mismatches.append((key, uval, cval))
    return matches, mismatches

def generate_report(urdf_data, index, out_dir):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    md_path = out_dir / f"integrity_report_{timestamp}.md"
    coverage_csv = out_dir / f"name_coverage_{timestamp}.csv"
    mismatch_csv = out_dir / f"detail_mismatches_{timestamp}.csv"

    # Name coverage
    coverage_rows = []
    mismatch_rows = []

    urdf_links = set(urdf_data["links"].keys())
    urdf_joints = set(urdf_data["joints"].keys())

    json_all_names = set()
    for rec in index["idents"]:
        ident = rec["ident"]
        for k in ("name", "idShort", "displayName"):
            v = ident.get(k)
            if isinstance(v, str):
                json_all_names.add(v)

    missing_links = sorted(list(urdf_links - json_all_names))
    missing_joints = sorted(list(urdf_joints - json_all_names))

    # Fill CSV coverage for links
    for lname in sorted(urdf_links):
        hits, text_hits = find_candidates_by_name(index, lname)
        coverage_rows.append({
            "type": "link",
            "name": lname,
            "found_in_idents": len(hits),
            "found_in_text": len(text_hits),
        })
        # try detailed comparisons if we have a candidate container
        for rec in hits:
            matches, mismatches = compare_link_inertial(urdf_data["links"][lname], rec["container"])
            for k, u, c in matches:
                mismatch_rows.append({"item_type":"link","name":lname,"field":k,"urdf":u,"json":c,"status":"match","file":rec["file"],"path":rec["path"]})
            for k, u, c in mismatches:
                mismatch_rows.append({"item_type":"link","name":lname,"field":k,"urdf":u,"json":c,"status":"MISMATCH","file":rec["file"],"path":rec["path"]})

    # Fill CSV coverage for joints
    for jname in sorted(urdf_joints):
        hits, text_hits = find_candidates_by_name(index, jname)
        coverage_rows.append({
            "type": "joint",
            "name": jname,
            "found_in_idents": len(hits),
            "found_in_text": len(text_hits),
        })
        for rec in hits:
            matches, mismatches = compare_joint_limits(urdf_data["joints"][jname], rec["container"])
            for k, u, c in matches:
                mismatch_rows.append({"item_type":"joint","name":jname,"field":k,"urdf":u,"json":c,"status":"match","file":rec["file"],"path":rec["path"]})
            for k, u, c in mismatches:
                mismatch_rows.append({"item_type":"joint","name":jname,"field":k,"urdf":u,"json":c,"status":"MISMATCH","file":rec["file"],"path":rec["path"]})

    # Write CSVs
    with open(coverage_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["type", "name", "found_in_idents", "found_in_text"])
        writer.writeheader()
        writer.writerows(coverage_rows)

    with open(mismatch_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["item_type","name","field","urdf","json","status","file","path"])
        writer.writeheader()
        writer.writerows(mismatch_rows)

    # Markdown summary
    lines = []
    lines.append(f"# URDF ↔ AAS JSON Integrity Report")
    lines.append("")
    lines.append(f"- Generated: {timestamp}")
    lines.append(f"- Robot: **{urdf_data['robot_name']}**")
    lines.append(f"- Links in URDF: {len(urdf_links)}")
    lines.append(f"- Joints in URDF: {len(urdf_joints)}")
    lines.append("")
    lines.append("## Coverage Summary")
    lines.append("")
    # Simple stats
    links_with_hits = [r for r in coverage_rows if r["type"]=="link" and (r["found_in_idents"]>0 or r["found_in_text"]>0)]
    joints_with_hits = [r for r in coverage_rows if r["type"]=="joint" and (r["found_in_idents"]>0 or r["found_in_text"]>0)]
    lines.append(f"- Links matched in JSON (by name/idShort/text): **{len(links_with_hits)}/{len(urdf_links)}**")
    lines.append(f"- Joints matched in JSON (by name/idShort/text): **{len(joints_with_hits)}/{len(urdf_joints)}**")
    lines.append("")
    if missing_links:
        lines.append("### URDF Links missing in JSON by identifier")
        for n in missing_links[:50]:
            lines.append(f"- {n}")
        if len(missing_links) > 50:
            lines.append(f"... and {len(missing_links)-50} more")
        lines.append("")
    if missing_joints:
        lines.append("### URDF Joints missing in JSON by identifier")
        for n in missing_joints[:50]:
            lines.append(f"- {n}")
        if len(missing_joints) > 50:
            lines.append(f"... and {len(missing_joints)-50} more")
        lines.append("")

    lines.append("## Detailed Numeric Comparisons")
    lines.append("Numeric comparisons (inertial & joint limits) found in candidates with matching names. See CSV for full details.")
    lines.append("")
    mismatch_count = sum(1 for r in mismatch_rows if r["status"]=="MISMATCH")
    match_count = sum(1 for r in mismatch_rows if r["status"]=="match")
    lines.append(f"- Matches: **{match_count}**")
    lines.append(f"- Mismatches: **{mismatch_count}**")
    lines.append("")
    lines.append(f"**Artifacts**:")
    lines.append(f"- Name coverage CSV: `{coverage_csv.name}`")
    lines.append(f"- Detailed mismatches CSV: `{mismatch_csv.name}`")

    md = "\n".join(lines)
    with open(md_path, "w", encoding="utf-8") as f:
        f.write(md)

    return {
        "markdown": str(md_path),
        "coverage_csv": str(coverage_csv),
        "mismatch_csv": str(mismatch_csv)
    }

def main():
    parser = argparse.ArgumentParser(description="Check integrity between URDF and AAS JSON submodels.")
    parser.add_argument("--urdf", required=True, help="Path to URDF file (.urdf)")
    parser.add_argument("--json-dir", required=True, help="Directory containing AAS JSON files")
    parser.add_argument("--out", default=".", help="Output directory for the report and CSVs")
    args = parser.parse_args()

    urdf_data = parse_urdf(args.urdf)
    index = load_json_dir(args.json_dir)
    artifacts = generate_report(urdf_data, index, args.out)

    print("Report generated:")
    print(artifacts["markdown"])
    print(artifacts["coverage_csv"])
    print(artifacts["mismatch_csv"])

if __name__ == "__main__":
    main()
