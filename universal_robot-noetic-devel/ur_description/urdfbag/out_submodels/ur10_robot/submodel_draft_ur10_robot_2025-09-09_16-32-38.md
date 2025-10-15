# Submodel 描述文稿（URDF 自动生成）

- 生成时间：2025-09-09 16:32:38
- 机器人：**ur10_robot**
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
- Link `shoulder_link`: mass=7.1, inertia={ ixx=0.03408, ixy=2e-05, ixz=-0.00425, iyy=0.03529, iyz=8e-05, izz=0.02156 }
  - 惯性原点：xyz=0.021 -0.027 0.0, rpy=1.570796326794897 0 0
- Link `upper_arm_link`: mass=12.7, inertia={ ixx=0.02814, ixy=5e-05, ixz=-0.01561, iyy=0.77068, iyz=2e-05, izz=0.76943 }
  - 惯性原点：xyz=-0.232 0.0 0.158, rpy=0 0 0
- Link `forearm_link`: mass=4.27, inertia={ ixx=0.01014, ixy=8e-05, ixz=0.00916, iyy=0.30928, iyz=0, izz=0.30646 }
  - 惯性原点：xyz=-0.3323 0.0 0.068, rpy=0 0 0
- Link `wrist_1_link`: mass=2, inertia={ ixx=0.00296, ixy=-1e-05, ixz=0, iyy=0.00222, iyz=-0.00024, izz=0.00258 }
  - 惯性原点：xyz=0.0 -0.018 0.007, rpy=1.570796326794897 0 0
- Link `wrist_2_link`: mass=2, inertia={ ixx=0.00296, ixy=-1e-05, ixz=0, iyy=0.00222, iyz=-0.00024, izz=0.00258 }
  - 惯性原点：xyz=0.0 0.018 -0.007, rpy=-1.570796326794897 0 0
- Link `wrist_3_link`: mass=0.365, inertia={ ixx=0.0004, ixy=0, ixz=0, iyy=0.00041, iyz=0, izz=0.00034 }
  - 惯性原点：xyz=0.0 0 -0.026, rpy=0 0 0
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
- Joint `wrist_1_joint` limits: { lower=-6.28319, upper=6.28319, effort=56, velocity=3.14159 }
- Joint `wrist_2_joint` limits: { lower=-6.28319, upper=6.28319, effort=56, velocity=3.14159 }
- Joint `wrist_3_joint` limits: { lower=-6.28319, upper=6.28319, effort=56, velocity=3.14159 }

### 4) ControlSubmodel
- Transmission `shoulder_pan_trans`: type=transmission_interface/SimpleTransmission, joints=['shoulder_pan_joint'], actuators=['shoulder_pan_motor']
- Transmission `shoulder_lift_trans`: type=transmission_interface/SimpleTransmission, joints=['shoulder_lift_joint'], actuators=['shoulder_lift_motor']
- Transmission `elbow_trans`: type=transmission_interface/SimpleTransmission, joints=['elbow_joint'], actuators=['elbow_motor']
- Transmission `wrist_1_trans`: type=transmission_interface/SimpleTransmission, joints=['wrist_1_joint'], actuators=['wrist_1_motor']
- Transmission `wrist_2_trans`: type=transmission_interface/SimpleTransmission, joints=['wrist_2_joint'], actuators=['wrist_2_motor']
- Transmission `wrist_3_trans`: type=transmission_interface/SimpleTransmission, joints=['wrist_3_joint'], actuators=['wrist_3_motor']

### 5) VisualizationSubmodel
- Link `base_link_inertia` → Visual: package://ur_description/meshes/ur10/visual/base.dae | Collision: package://ur_description/meshes/ur10/collision/base.stl | Materials: LightGrey
- Link `shoulder_link` → Visual: package://ur_description/meshes/ur10/visual/shoulder.dae | Collision: package://ur_description/meshes/ur10/collision/shoulder.stl | Materials: LightGrey
- Link `upper_arm_link` → Visual: package://ur_description/meshes/ur10/visual/upperarm.dae | Collision: package://ur_description/meshes/ur10/collision/upperarm.stl | Materials: LightGrey
- Link `forearm_link` → Visual: package://ur_description/meshes/ur10/visual/forearm.dae | Collision: package://ur_description/meshes/ur10/collision/forearm.stl | Materials: LightGrey
- Link `wrist_1_link` → Visual: package://ur_description/meshes/ur10/visual/wrist1.dae | Collision: package://ur_description/meshes/ur10/collision/wrist1.stl | Materials: LightGrey
- Link `wrist_2_link` → Visual: package://ur_description/meshes/ur10/visual/wrist2.dae | Collision: package://ur_description/meshes/ur10/collision/wrist2.stl | Materials: LightGrey
- Link `wrist_3_link` → Visual: package://ur_description/meshes/ur10/visual/wrist3.dae | Collision: package://ur_description/meshes/ur10/collision/wrist3.stl | Materials: LightGrey

## ConceptDescription（建议词条草案）
- jointLowerLimit / jointUpperLimit / jointMaxVelocity / jointMaxEffort
- linkMass / linkInertiaIxx/Iyy/Izz/Ixy/Ixz/Iyz
- visualMeshUri / collisionMeshUri / materialName
- transmissionType / actuatorName / transmissionJointRef
