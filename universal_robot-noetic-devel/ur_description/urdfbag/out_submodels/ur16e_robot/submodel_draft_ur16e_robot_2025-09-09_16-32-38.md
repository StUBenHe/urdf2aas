# Submodel 描述文稿（URDF 自动生成）

- 生成时间：2025-09-09 16:32:38
- 机器人：**ur16e_robot**
- 链接（links）：11 个
- 关节（joints）：10 个
- 传动（transmissions）：6 个
- 材质（materials）：0 种

---

## 建议的 AAS Submodels（草案）

### 1) StructureSubmodel
- Link `base_link`: mass=—
- Link `base_link_inertia`: mass=4, inertia={ ixx=0.00610633, ixy=0, ixz=0, iyy=0.00610633, iyz=0, izz=0.01125 }
  - 惯性原点：xyz=0 0 0, rpy=0 0 0
- Link `shoulder_link`: mass=7.369, inertia={ ixx=0.03351, ixy=2e-05, ixz=-1e-05, iyy=0.03374, iyz=0.00374, izz=0.021 }
  - 惯性原点：xyz=0.0 -0.03 -0.016, rpy=1.570796326794897 0 0
- Link `upper_arm_link`: mass=10.45, inertia={ ixx=0.02796, ixy=-0.0001, ixz=-0.0072, iyy=0.47558, iyz=3e-05, izz=0.47635 }
  - 惯性原点：xyz=-0.1764 0.0 0.16, rpy=0 0 0
- Link `forearm_link`: mass=4.321, inertia={ ixx=0.01091, ixy=6e-05, ixz=0.01012, iyy=0.1206, iyz=1e-05, izz=0.11714 }
  - 惯性原点：xyz=-0.166 0.0 0.065, rpy=0 0 0
- Link `wrist_1_link`: mass=2.18, inertia={ ixx=0.00609, ixy=-1e-05, ixz=0, iyy=0.00245, iyz=0.00083, izz=0.00579 }
  - 惯性原点：xyz=0.0 -0.011 -0.009, rpy=1.570796326794897 0 0
- Link `wrist_2_link`: mass=2.033, inertia={ ixx=0.00389, ixy=-1e-05, ixz=0, iyy=0.00219, iyz=-0.00045, izz=0.00363 }
  - 惯性原点：xyz=0.0 0.012 -0.018, rpy=-1.570796326794897 0 0
- Link `wrist_3_link`: mass=0.907, inertia={ ixx=0.00117, ixy=0, ixz=0, iyy=0.00118, iyz=0, izz=0.00084 }
  - 惯性原点：xyz=0.0 0.0 -0.044, rpy=0 0 0
- Link `base`: mass=—
- Link `flange`: mass=—
- Link `tool0`: mass=—

### 2) KinematicsSubmodel
- Joint `base_link-base_link_inertia` (fixed): base_link → base_link_inertia, axis=—
- Joint `shoulder_pan_joint` (revolute): base_link_inertia → shoulder_link, axis=0 0 1
- Joint `shoulder_lift_joint` (revolute): shoulder_link → upper_arm_link, axis=0 0 1
- Joint `elbow_joint` (revolute): upper_arm_link → forearm_link, axis=0 0 1
- Joint `wrist_1_joint` (revolute): forearm_link → wrist_1_link, axis=0 0 1
- Joint `wrist_2_joint` (revolute): wrist_1_link → wrist_2_link, axis=0 0 1
- Joint `wrist_3_joint` (revolute): wrist_2_link → wrist_3_link, axis=0 0 1
- Joint `base_link-base_fixed_joint` (fixed): base_link → base, axis=—
- Joint `wrist_3-flange` (fixed): wrist_3_link → flange, axis=—
- Joint `flange-tool0` (fixed): flange → tool0, axis=—

### 3) DynamicsSubmodel
- Joint `shoulder_pan_joint` limits: { lower=-6.28319, upper=6.28319, effort=330, velocity=2.0944 }
- Joint `shoulder_lift_joint` limits: { lower=-6.28319, upper=6.28319, effort=330, velocity=2.0944 }
- Joint `elbow_joint` limits: { lower=-3.14159, upper=3.14159, effort=150, velocity=3.14159 }
- Joint `wrist_1_joint` limits: { lower=-6.28319, upper=6.28319, effort=54, velocity=3.14159 }
- Joint `wrist_2_joint` limits: { lower=-6.28319, upper=6.28319, effort=54, velocity=3.14159 }
- Joint `wrist_3_joint` limits: { lower=-6.28319, upper=6.28319, effort=54, velocity=3.14159 }

### 4) ControlSubmodel
- Transmission `shoulder_pan_trans`: type=transmission_interface/SimpleTransmission, joints=['shoulder_pan_joint'], actuators=['shoulder_pan_motor']
- Transmission `shoulder_lift_trans`: type=transmission_interface/SimpleTransmission, joints=['shoulder_lift_joint'], actuators=['shoulder_lift_motor']
- Transmission `elbow_trans`: type=transmission_interface/SimpleTransmission, joints=['elbow_joint'], actuators=['elbow_motor']
- Transmission `wrist_1_trans`: type=transmission_interface/SimpleTransmission, joints=['wrist_1_joint'], actuators=['wrist_1_motor']
- Transmission `wrist_2_trans`: type=transmission_interface/SimpleTransmission, joints=['wrist_2_joint'], actuators=['wrist_2_motor']
- Transmission `wrist_3_trans`: type=transmission_interface/SimpleTransmission, joints=['wrist_3_joint'], actuators=['wrist_3_motor']

### 5) VisualizationSubmodel
- Link `base_link_inertia` → Visual: package://ur_description/meshes/ur10e/visual/base.dae | Collision: package://ur_description/meshes/ur10e/collision/base.stl | Materials: LightGrey
- Link `shoulder_link` → Visual: package://ur_description/meshes/ur10e/visual/shoulder.dae | Collision: package://ur_description/meshes/ur10e/collision/shoulder.stl | Materials: LightGrey
- Link `upper_arm_link` → Visual: package://ur_description/meshes/ur16e/visual/upperarm.dae | Collision: package://ur_description/meshes/ur16e/collision/upperarm.stl | Materials: LightGrey
- Link `forearm_link` → Visual: package://ur_description/meshes/ur16e/visual/forearm.dae | Collision: package://ur_description/meshes/ur16e/collision/forearm.stl | Materials: LightGrey
- Link `wrist_1_link` → Visual: package://ur_description/meshes/ur10e/visual/wrist1.dae | Collision: package://ur_description/meshes/ur10e/collision/wrist1.stl | Materials: LightGrey
- Link `wrist_2_link` → Visual: package://ur_description/meshes/ur10e/visual/wrist2.dae | Collision: package://ur_description/meshes/ur10e/collision/wrist2.stl | Materials: LightGrey
- Link `wrist_3_link` → Visual: package://ur_description/meshes/ur10e/visual/wrist3.dae | Collision: package://ur_description/meshes/ur10e/collision/wrist3.stl | Materials: LightGrey

## ConceptDescription（建议词条草案）
- jointLowerLimit / jointUpperLimit / jointMaxVelocity / jointMaxEffort
- linkMass / linkInertiaIxx/Iyy/Izz/Ixy/Ixz/Iyz
- visualMeshUri / collisionMeshUri / materialName
- transmissionType / actuatorName / transmissionJointRef
