from pathlib import Path
from urdfpy import URDF
import json

# === 1. 加载 URDF 文件 ===
urdf_path = Path("ur3.urdf")  # 改为你的文件路径
robot = URDF.load(urdf_path)

# === 2. 生成各个 Submodel ===

# 1️⃣ IdentificationSubmodel
identification_submodel = {
    "id": "IdentificationSubmodel",
    "manufacturer": "Universal Robots",
    "model": robot.name,
    "serialNumber": f"{robot.name.upper()}-001",
    "version": "1.0"
}

# 2️⃣ StructureSubmodel
structure_submodel = {
    "id": "StructureSubmodel",
    "links": [link.name for link in robot.links],
    "joints": [
        {
            "name": joint.name,
            "type": joint.joint_type,
            "parent": joint.parent,
            "child": joint.child
        }
        for joint in robot.joints
    ]
}

# 3️⃣ ControlSubmodel
control_submodel = {
    "id": "ControlSubmodel",
    "controlJoints": [
        {
            "name": joint.name,
            "positionLimits": list(joint.limit.position) if joint.limit and joint.limit.position else None,
            "velocityLimit": joint.limit.velocity if joint.limit and joint.limit.velocity else None
        }
        for joint in robot.joints if joint.joint_type in ["revolute", "prismatic"]
    ],
    "controllerType": "Position"
}

# 4️⃣ KinematicsSubmodel
kinematics_submodel = {
    "id": "KinematicsSubmodel",
    "kinematicChain": [
        {
            "joint": joint.name,
            "axis": joint.axis.tolist() if joint.axis is not None else None,
            "origin": {
                "xyz": joint.origin[:3].tolist(),
                "rpy": joint.origin[3:].tolist()
            } if joint.origin is not None else None
        }
        for joint in robot.joints
    ]
}

# 5️⃣ DynamicsSubmodel
dynamics_submodel = {
    "id": "DynamicsSubmodel",
    "dynamics": [
        {
            "link": link.name,
            "mass": link.inertial.mass if link.inertial else None,
            "inertia": link.inertial.inertia if link.inertial else None
        }
        for link in robot.links
    ]
}

# 6️⃣ SafetySubmodel
safety_submodel = {
    "id": "SafetySubmodel",
    "collisionChecking": True,
    "jointSoftLimits": True,
    "emergencyStop": "Not Specified"
}

# 7️⃣ VisualizationSubmodel
visualization_submodel = {
    "id": "VisualizationSubmodel",
    "visualElements": [
        {
            "link": link.name,
            "geometry": visual.geometry.mesh.filename if visual.geometry.mesh else "primitive",
            "origin": {
                "xyz": visual.origin[:3].tolist(),
                "rpy": visual.origin[3:].tolist()
            } if visual.origin is not None else None
        }
        for link in robot.links for visual in link.visuals
    ]
}

# 8️⃣ MaintenanceSubmodel
maintenance_submodel = {
    "id": "MaintenanceSubmodel",
    "recommendedIntervalHours": 5000,
    "lastServiceDate": "2025-01-01",
    "serviceNotes": "Factory default, not yet serviced."
}

# === 3. 保存所有 Submodel ===
aas_submodels = {
    "IdentificationSubmodel": identification_submodel,
    "StructureSubmodel": structure_submodel,
    "ControlSubmodel": control_submodel,
    "KinematicsSubmodel": kinematics_submodel,
    "DynamicsSubmodel": dynamics_submodel,
    "SafetySubmodel": safety_submodel,
    "VisualizationSubmodel": visualization_submodel,
    "MaintenanceSubmodel": maintenance_submodel
}

output_dir = Path("aas_submodels")
output_dir.mkdir(exist_ok=True)

for name, data in aas_submodels.items():
    with open(output_dir / f"{name}.json", "w") as f:
        json.dump(data, f, indent=2)

print(f"✅ 所有 Submodel 已保存到 {output_dir}")
