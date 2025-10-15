# Submodel 描述文稿（URDF 自动生成）

- 生成时间：2025-09-09 16:32:38
- 机器人：**ur20_robot**
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
- Link `shoulder_link`: mass=16.343, inertia={ ixx=0.08866, ixy=-0.00011, ixz=-0.00011, iyy=0.07632, iyz=0.0072, izz=0.08418 }
  - 惯性原点：xyz=0.0 -0.0062 -0.061, rpy=1.570796326794897 0 0
- Link `upper_arm_link`: mass=29.632, inertia={ ixx=0.1467, ixy=0.00019, ixz=-0.05163, iyy=4.6659, iyz=4e-05, izz=4.6348 }
  - 惯性原点：xyz=-0.3394 0.0 0.2098, rpy=0 0 0
- Link `forearm_link`: mass=7.879, inertia={ ixx=0.02612, ixy=-5e-05, ixz=-0.02898, iyy=0.75763, iyz=-1e-05, izz=0.75327 }
  - 惯性原点：xyz=-0.4053 0.0 0.0604, rpy=0 0 0
- Link `wrist_1_link`: mass=3.054, inertia={ ixx=0.00555, ixy=-1e-05, ixz=-2e-05, iyy=0.00537, iyz=0.00036, izz=0.00402 }
  - 惯性原点：xyz=0.0 -0.0393 -0.0026, rpy=1.570796326794897 0 0
- Link `wrist_2_link`: mass=3.126, inertia={ ixx=0.00586, ixy=-1e-05, ixz=-2e-05, iyy=0.00578, iyz=-0.00037, izz=0.00427 }
  - 惯性原点：xyz=0.0 0.0379 -0.0024, rpy=-1.570796326794897 0 0
- Link `wrist_3_link`: mass=0.926, inertia={ ixx=0.00092, ixy=0, ixz=0, iyy=0.00091, iyz=0, izz=0.00117 }
  - 惯性原点：xyz=0.0 0.0 -0.0293, rpy=0 0 0
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
- Joint `shoulder_pan_joint` limits: { lower=-6.28319, upper=6.28319, effort=738, velocity=2.0944 }
- Joint `shoulder_lift_joint` limits: { lower=-6.28319, upper=6.28319, effort=738, velocity=2.0944 }
- Joint `elbow_joint` limits: { lower=-3.14159, upper=3.14159, effort=433, velocity=2.61799 }
- Joint `wrist_1_joint` limits: { lower=-6.28319, upper=6.28319, effort=107, velocity=3.66519 }
- Joint `wrist_2_joint` limits: { lower=-6.28319, upper=6.28319, effort=107, velocity=3.66519 }
- Joint `wrist_3_joint` limits: { lower=-6.28319, upper=6.28319, effort=107, velocity=3.66519 }

### 4) ControlSubmodel
- Transmission `shoulder_pan_trans`: type=transmission_interface/SimpleTransmission, joints=['shoulder_pan_joint'], actuators=['shoulder_pan_motor']
- Transmission `shoulder_lift_trans`: type=transmission_interface/SimpleTransmission, joints=['shoulder_lift_joint'], actuators=['shoulder_lift_motor']
- Transmission `elbow_trans`: type=transmission_interface/SimpleTransmission, joints=['elbow_joint'], actuators=['elbow_motor']
- Transmission `wrist_1_trans`: type=transmission_interface/SimpleTransmission, joints=['wrist_1_joint'], actuators=['wrist_1_motor']
- Transmission `wrist_2_trans`: type=transmission_interface/SimpleTransmission, joints=['wrist_2_joint'], actuators=['wrist_2_motor']
- Transmission `wrist_3_trans`: type=transmission_interface/SimpleTransmission, joints=['wrist_3_joint'], actuators=['wrist_3_motor']

### 5) VisualizationSubmodel
- Link `base_link_inertia` → Visual: package://ur_description/meshes/ur20/visual/base.dae | Collision: package://ur_description/meshes/ur20/collision/base.stl | Materials: LightGrey
- Link `shoulder_link` → Visual: package://ur_description/meshes/ur20/visual/shoulder.dae | Collision: package://ur_description/meshes/ur20/collision/shoulder.stl | Materials: LightGrey
- Link `upper_arm_link` → Visual: package://ur_description/meshes/ur20/visual/upperarm.dae | Collision: package://ur_description/meshes/ur20/collision/upperarm.stl | Materials: LightGrey
- Link `forearm_link` → Visual: package://ur_description/meshes/ur20/visual/forearm.dae | Collision: package://ur_description/meshes/ur20/collision/forearm.stl | Materials: LightGrey
- Link `wrist_1_link` → Visual: package://ur_description/meshes/ur20/visual/wrist1.dae | Collision: package://ur_description/meshes/ur20/collision/wrist1.stl | Materials: LightGrey
- Link `wrist_2_link` → Visual: package://ur_description/meshes/ur20/visual/wrist2.dae | Collision: package://ur_description/meshes/ur20/collision/wrist2.stl | Materials: LightGrey
- Link `wrist_3_link` → Visual: package://ur_description/meshes/ur20/visual/wrist3.dae | Collision: package://ur_description/meshes/ur20/collision/wrist3.stl | Materials: LightGrey

## ConceptDescription（建议词条草案）
- jointLowerLimit / jointUpperLimit / jointMaxVelocity / jointMaxEffort
- linkMass / linkInertiaIxx/Iyy/Izz/Ixy/Ixz/Iyz
- visualMeshUri / collisionMeshUri / materialName
- transmissionType / actuatorName / transmissionJointRef
