# Submodel 描述文稿（URDF 自动生成）

- 生成时间：2025-09-09 16:32:38
- 机器人：**ur3e_robot**
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
- Link `shoulder_link`: mass=1.98, inertia={ ixx=0.0027657, ixy=9.6e-06, ixz=-1.49e-05, iyy=0.0021501, iyz=0.0001873, izz=0.0025626 }
  - 惯性原点：xyz=0.0 0.0 -0.02, rpy=1.570796326794897 0 0
- Link `upper_arm_link`: mass=3.4445, inertia={ ixx=0.0042623, ixy=-4.09e-05, ixz=-0.001496, iyy=0.0460284, iyz=-1.29e-05, izz=0.0448947 }
  - 惯性原点：xyz=-0.11355 0.0 0.1157, rpy=0 0 0
- Link `forearm_link`: mass=1.437, inertia={ ixx=0.0010511, ixy=0, ixz=-3.31e-05, iyy=0.0113396, iyz=2.2e-06, izz=0.0110895 }
  - 惯性原点：xyz=-0.1632 0.0 0.0238, rpy=0 0 0
- Link `wrist_1_link`: mass=0.871, inertia={ ixx=0.0008174, ixy=-5e-07, ixz=2.1e-06, iyy=0.0006844, iyz=6.17e-05, izz=0.0005497 }
  - 惯性原点：xyz=0.0 -0.01 0.0, rpy=1.570796326794897 0 0
- Link `wrist_2_link`: mass=0.805, inertia={ ixx=0.0006059, ixy=-6e-07, ixz=-6e-07, iyy=0.0005837, iyz=-2.71e-05, izz=0.0003831 }
  - 惯性原点：xyz=0.0 0.01 0.0, rpy=-1.570796326794897 0 0
- Link `wrist_3_link`: mass=0.261, inertia={ ixx=0.000108, ixy=-1e-07, ixz=-1e-07, iyy=0.0001086, iyz=-2e-07, izz=0.0001371 }
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
- Joint `shoulder_pan_joint` limits: { lower=-6.28319, upper=6.28319, effort=54, velocity=3.14159 }
- Joint `shoulder_lift_joint` limits: { lower=-6.28319, upper=6.28319, effort=54, velocity=3.14159 }
- Joint `elbow_joint` limits: { lower=-3.14159, upper=3.14159, effort=28, velocity=3.14159 }
- Joint `wrist_1_joint` limits: { lower=-6.28319, upper=6.28319, effort=9, velocity=6.28319 }
- Joint `wrist_2_joint` limits: { lower=-6.28319, upper=6.28319, effort=9, velocity=6.28319 }
- Joint `wrist_3_joint` limits: { lower=-6.28319, upper=6.28319, effort=9, velocity=6.28319 }

### 4) ControlSubmodel
- Transmission `shoulder_pan_trans`: type=transmission_interface/SimpleTransmission, joints=['shoulder_pan_joint'], actuators=['shoulder_pan_motor']
- Transmission `shoulder_lift_trans`: type=transmission_interface/SimpleTransmission, joints=['shoulder_lift_joint'], actuators=['shoulder_lift_motor']
- Transmission `elbow_trans`: type=transmission_interface/SimpleTransmission, joints=['elbow_joint'], actuators=['elbow_motor']
- Transmission `wrist_1_trans`: type=transmission_interface/SimpleTransmission, joints=['wrist_1_joint'], actuators=['wrist_1_motor']
- Transmission `wrist_2_trans`: type=transmission_interface/SimpleTransmission, joints=['wrist_2_joint'], actuators=['wrist_2_motor']
- Transmission `wrist_3_trans`: type=transmission_interface/SimpleTransmission, joints=['wrist_3_joint'], actuators=['wrist_3_motor']

### 5) VisualizationSubmodel
- Link `base_link_inertia` → Visual: package://ur_description/meshes/ur3e/visual/base.dae | Collision: package://ur_description/meshes/ur3e/collision/base.stl | Materials: LightGrey
- Link `shoulder_link` → Visual: package://ur_description/meshes/ur3e/visual/shoulder.dae | Collision: package://ur_description/meshes/ur3e/collision/shoulder.stl | Materials: LightGrey
- Link `upper_arm_link` → Visual: package://ur_description/meshes/ur3e/visual/upperarm.dae | Collision: package://ur_description/meshes/ur3e/collision/upperarm.stl | Materials: LightGrey
- Link `forearm_link` → Visual: package://ur_description/meshes/ur3e/visual/forearm.dae | Collision: package://ur_description/meshes/ur3e/collision/forearm.stl | Materials: LightGrey
- Link `wrist_1_link` → Visual: package://ur_description/meshes/ur3e/visual/wrist1.dae | Collision: package://ur_description/meshes/ur3e/collision/wrist1.stl | Materials: LightGrey
- Link `wrist_2_link` → Visual: package://ur_description/meshes/ur3e/visual/wrist2.dae | Collision: package://ur_description/meshes/ur3e/collision/wrist2.stl | Materials: LightGrey
- Link `wrist_3_link` → Visual: package://ur_description/meshes/ur3e/visual/wrist3.dae | Collision: package://ur_description/meshes/ur3e/collision/wrist3.stl | Materials: LightGrey

## ConceptDescription（建议词条草案）
- jointLowerLimit / jointUpperLimit / jointMaxVelocity / jointMaxEffort
- linkMass / linkInertiaIxx/Iyy/Izz/Ixy/Ixz/Iyz
- visualMeshUri / collisionMeshUri / materialName
- transmissionType / actuatorName / transmissionJointRef
