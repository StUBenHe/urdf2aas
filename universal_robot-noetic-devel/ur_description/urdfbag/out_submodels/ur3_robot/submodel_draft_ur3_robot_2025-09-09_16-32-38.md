# Submodel 描述文稿（URDF 自动生成）

- 生成时间：2025-09-09 16:32:38
- 机器人：**ur3_robot**
- 链接（links）：11 个
- 关节（joints）：10 个
- 传动（transmissions）：6 个
- 材质（materials）：0 种

---

## 建议的 AAS Submodels（草案）

### 1) StructureSubmodel
- Link `base_link`: mass=—
- Link `base_link_inertia`: mass=2, inertia={ ixx=0.00305317, ixy=0, ixz=0, iyy=0.00305317, iyz=0, izz=0.005625 }
  - 惯性原点：xyz=0 0 0, rpy=0 0 0
- Link `shoulder_link`: mass=2, inertia={ ixx=0.00809317, ixy=0, ixz=0, iyy=0.00809317, iyz=0, izz=0.005625 }
  - 惯性原点：xyz=0.0 0.0 -0.02, rpy=0 0 0
- Link `upper_arm_link`: mass=3.42, inertia={ ixx=0.0217285, ixy=0, ixz=0, iyy=0.0217285, iyz=0, izz=0.00961875 }
  - 惯性原点：xyz=-0.11365 0.0 0.1157, rpy=0 1.570796326794897 0
- Link `forearm_link`: mass=1.26, inertia={ ixx=0.00654681, ixy=0, ixz=0, iyy=0.00654681, iyz=0, izz=0.00354375 }
  - 惯性原点：xyz=-0.16325 0.0 0.0238, rpy=0 1.570796326794897 0
- Link `wrist_1_link`: mass=0.8, inertia={ ixx=0.00161064, ixy=0, ixz=0, iyy=0.00161064, iyz=0, izz=0.00225 }
  - 惯性原点：xyz=0.0 -0.01 0.0, rpy=0 0 0
- Link `wrist_2_link`: mass=0.8, inertia={ ixx=0.00157217, ixy=0, ixz=0, iyy=0.00157217, iyz=0, izz=0.00225 }
  - 惯性原点：xyz=0.0 0.01 0.0, rpy=0 0 0
- Link `wrist_3_link`: mass=0.35, inertia={ ixx=0.000136267, ixy=0, ixz=0, iyy=0.000136267, iyz=0, izz=0.0001792 }
  - 惯性原点：xyz=0.0 0.0 -0.02, rpy=0 0 0
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
- Joint `shoulder_pan_joint` limits: { lower=-6.28319, upper=6.28319, effort=56, velocity=3.14159 }
- Joint `shoulder_lift_joint` limits: { lower=-6.28319, upper=6.28319, effort=56, velocity=3.14159 }
- Joint `elbow_joint` limits: { lower=-3.14159, upper=3.14159, effort=28, velocity=3.14159 }
- Joint `wrist_1_joint` limits: { lower=-6.28319, upper=6.28319, effort=12, velocity=6.28319 }
- Joint `wrist_2_joint` limits: { lower=-6.28319, upper=6.28319, effort=12, velocity=6.28319 }
- Joint `wrist_3_joint` limits: { lower=-6.28319, upper=6.28319, effort=12, velocity=6.28319 }

### 4) ControlSubmodel
- Transmission `shoulder_pan_trans`: type=transmission_interface/SimpleTransmission, joints=['shoulder_pan_joint'], actuators=['shoulder_pan_motor']
- Transmission `shoulder_lift_trans`: type=transmission_interface/SimpleTransmission, joints=['shoulder_lift_joint'], actuators=['shoulder_lift_motor']
- Transmission `elbow_trans`: type=transmission_interface/SimpleTransmission, joints=['elbow_joint'], actuators=['elbow_motor']
- Transmission `wrist_1_trans`: type=transmission_interface/SimpleTransmission, joints=['wrist_1_joint'], actuators=['wrist_1_motor']
- Transmission `wrist_2_trans`: type=transmission_interface/SimpleTransmission, joints=['wrist_2_joint'], actuators=['wrist_2_motor']
- Transmission `wrist_3_trans`: type=transmission_interface/SimpleTransmission, joints=['wrist_3_joint'], actuators=['wrist_3_motor']

### 5) VisualizationSubmodel
- Link `base_link_inertia` → Visual: package://ur_description/meshes/ur3/visual/base.dae | Collision: package://ur_description/meshes/ur3/collision/base.stl | Materials: LightGrey
- Link `shoulder_link` → Visual: package://ur_description/meshes/ur3/visual/shoulder.dae | Collision: package://ur_description/meshes/ur3/collision/shoulder.stl | Materials: LightGrey
- Link `upper_arm_link` → Visual: package://ur_description/meshes/ur3/visual/upperarm.dae | Collision: package://ur_description/meshes/ur3/collision/upperarm.stl | Materials: LightGrey
- Link `forearm_link` → Visual: package://ur_description/meshes/ur3/visual/forearm.dae | Collision: package://ur_description/meshes/ur3/collision/forearm.stl | Materials: LightGrey
- Link `wrist_1_link` → Visual: package://ur_description/meshes/ur3/visual/wrist1.dae | Collision: package://ur_description/meshes/ur3/collision/wrist1.stl | Materials: LightGrey
- Link `wrist_2_link` → Visual: package://ur_description/meshes/ur3/visual/wrist2.dae | Collision: package://ur_description/meshes/ur3/collision/wrist2.stl | Materials: LightGrey
- Link `wrist_3_link` → Visual: package://ur_description/meshes/ur3/visual/wrist3.dae | Collision: package://ur_description/meshes/ur3/collision/wrist3.stl | Materials: LightGrey

## ConceptDescription（建议词条草案）
- jointLowerLimit / jointUpperLimit / jointMaxVelocity / jointMaxEffort
- linkMass / linkInertiaIxx/Iyy/Izz/Ixy/Ixz/Iyz
- visualMeshUri / collisionMeshUri / materialName
- transmissionType / actuatorName / transmissionJointRef
