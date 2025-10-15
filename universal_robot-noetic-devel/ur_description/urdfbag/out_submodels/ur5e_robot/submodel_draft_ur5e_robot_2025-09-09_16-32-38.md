# Submodel 描述文稿（URDF 自动生成）

- 生成时间：2025-09-09 16:32:38
- 机器人：**ur5e_robot**
- 链接（links）：11 个
- 关节（joints）：10 个
- 传动（transmissions）：6 个
- 材质（materials）：0 种

---

## 建议的 AAS Submodels（草案）

### 1) StructureSubmodel
- Link `base_link`: mass=—
- Link `base_link_inertia`: mass=4, inertia={ ixx=0.00443333, ixy=0, ixz=0, iyy=0.00443333, iyz=0, izz=0.0072 }
  - 惯性原点：xyz=0 0 0, rpy=0 0 0
- Link `shoulder_link`: mass=3.761, inertia={ ixx=0.0070021, ixy=7.3e-07, ixz=-1.053e-05, iyy=0.00648091, iyz=0.00049994, izz=0.00657286 }
  - 惯性原点：xyz=0.0 -0.00193 -0.02561, rpy=1.570796326794897 0 0
- Link `upper_arm_link`: mass=8.058, inertia={ ixx=0.0150589, ixy=-5.4e-05, ixz=5.63e-06, iyy=0.333881, iyz=-1.81e-06, izz=0.332472 }
  - 惯性原点：xyz=-0.2125 0.0 0.11336, rpy=0 0 0
- Link `forearm_link`: mass=2.846, inertia={ ixx=0.00399632, ixy=-1.365e-05, ixz=0.00137272, iyy=0.0787925, iyz=-6.6e-06, izz=0.0784851 }
  - 惯性原点：xyz=-0.2422 0.0 0.0265, rpy=0 0 0
- Link `wrist_1_link`: mass=1.37, inertia={ ixx=0.00165491, ixy=-2.82e-06, ixz=-4.38e-06, iyy=0.00135962, iyz=0.00010157, izz=0.00126279 }
  - 惯性原点：xyz=0.0 -0.01634 -0.0018, rpy=1.570796326794897 0 0
- Link `wrist_2_link`: mass=1.3, inertia={ ixx=0.00135617, ixy=-2.74e-06, ixz=4.44e-06, iyy=0.00127827, iyz=-5.048e-05, izz=0.00096614 }
  - 惯性原点：xyz=0.0 0.01634 -0.0018, rpy=-1.570796326794897 0 0
- Link `wrist_3_link`: mass=0.365, inertia={ ixx=0.00018694, ixy=6e-08, ixz=-1.7e-07, iyy=0.00018908, iyz=-9.2e-07, izz=0.00025756 }
  - 惯性原点：xyz=0.0 0.0 -0.001159, rpy=0 0 0
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
- Joint `shoulder_pan_joint` limits: { lower=-6.28319, upper=6.28319, effort=150, velocity=3.14159 }
- Joint `shoulder_lift_joint` limits: { lower=-6.28319, upper=6.28319, effort=150, velocity=3.14159 }
- Joint `elbow_joint` limits: { lower=-3.14159, upper=3.14159, effort=150, velocity=3.14159 }
- Joint `wrist_1_joint` limits: { lower=-6.28319, upper=6.28319, effort=28, velocity=3.14159 }
- Joint `wrist_2_joint` limits: { lower=-6.28319, upper=6.28319, effort=28, velocity=3.14159 }
- Joint `wrist_3_joint` limits: { lower=-6.28319, upper=6.28319, effort=28, velocity=3.14159 }

### 4) ControlSubmodel
- Transmission `shoulder_pan_trans`: type=transmission_interface/SimpleTransmission, joints=['shoulder_pan_joint'], actuators=['shoulder_pan_motor']
- Transmission `shoulder_lift_trans`: type=transmission_interface/SimpleTransmission, joints=['shoulder_lift_joint'], actuators=['shoulder_lift_motor']
- Transmission `elbow_trans`: type=transmission_interface/SimpleTransmission, joints=['elbow_joint'], actuators=['elbow_motor']
- Transmission `wrist_1_trans`: type=transmission_interface/SimpleTransmission, joints=['wrist_1_joint'], actuators=['wrist_1_motor']
- Transmission `wrist_2_trans`: type=transmission_interface/SimpleTransmission, joints=['wrist_2_joint'], actuators=['wrist_2_motor']
- Transmission `wrist_3_trans`: type=transmission_interface/SimpleTransmission, joints=['wrist_3_joint'], actuators=['wrist_3_motor']

### 5) VisualizationSubmodel
- Link `base_link_inertia` → Visual: package://ur_description/meshes/ur5e/visual/base.dae | Collision: package://ur_description/meshes/ur5e/collision/base.stl | Materials: LightGrey
- Link `shoulder_link` → Visual: package://ur_description/meshes/ur5e/visual/shoulder.dae | Collision: package://ur_description/meshes/ur5e/collision/shoulder.stl | Materials: LightGrey
- Link `upper_arm_link` → Visual: package://ur_description/meshes/ur5e/visual/upperarm.dae | Collision: package://ur_description/meshes/ur5e/collision/upperarm.stl | Materials: LightGrey
- Link `forearm_link` → Visual: package://ur_description/meshes/ur5e/visual/forearm.dae | Collision: package://ur_description/meshes/ur5e/collision/forearm.stl | Materials: LightGrey
- Link `wrist_1_link` → Visual: package://ur_description/meshes/ur5e/visual/wrist1.dae | Collision: package://ur_description/meshes/ur5e/collision/wrist1.stl | Materials: LightGrey
- Link `wrist_2_link` → Visual: package://ur_description/meshes/ur5e/visual/wrist2.dae | Collision: package://ur_description/meshes/ur5e/collision/wrist2.stl | Materials: LightGrey
- Link `wrist_3_link` → Visual: package://ur_description/meshes/ur5e/visual/wrist3.dae | Collision: package://ur_description/meshes/ur5e/collision/wrist3.stl | Materials: LightGrey

## ConceptDescription（建议词条草案）
- jointLowerLimit / jointUpperLimit / jointMaxVelocity / jointMaxEffort
- linkMass / linkInertiaIxx/Iyy/Izz/Ixy/Ixz/Iyz
- visualMeshUri / collisionMeshUri / materialName
- transmissionType / actuatorName / transmissionJointRef
