# Submodel 描述文稿（URDF 自动生成）

- 生成时间：2025-09-09 16:32:38
- 机器人：**ur15_robot**
- 链接（links）：11 个
- 关节（joints）：10 个
- 传动（transmissions）：6 个
- 材质（materials）：0 种

---

## 建议的 AAS Submodels（草案）

### 1) StructureSubmodel
- Link `base_link`: mass=—
- Link `base_link_inertia`: mass=4, inertia={ ixx=0.0322333, ixy=0, ixz=0, iyy=0.0322333, iyz=0, izz=0.0578 }
  - 惯性原点：xyz=0 0 0, rpy=0 0 0
- Link `shoulder_link`: mass=9.9883, inertia={ ixx=0.051334, ixy=2.5e-05, ixz=-1.6e-05, iyy=0.047702, iyz=0.008805, izz=0.034263 }
  - 惯性原点：xyz=2.4e-05 -0.025304 -0.033309, rpy=1.570796326794897 0 0
- Link `upper_arm_link`: mass=14.9255, inertia={ ixx=0.048202, ixy=0.000232, ixz=-0.032135, iyy=1.1885, iyz=-1.7e-05, izz=1.18277 }
  - 惯性原点：xyz=-0.228491 -8.3e-05 0.192787, rpy=0 0 0
- Link `forearm_link`: mass=6.1015, inertia={ ixx=0.018481, ixy=-5e-06, ixz=0.013601, iyy=0.303502, iyz=3e-06, izz=0.296843 }
  - 惯性原点：xyz=-0.210929 1.9e-05 0.070552, rpy=0 0 0
- Link `wrist_1_link`: mass=2.089, inertia={ ixx=0.004339, ixy=1.9e-05, ixz=1e-06, iyy=0.002548, iyz=0.000706, izz=0.003912 }
  - 惯性原点：xyz=2.5e-05 -0.016413 -0.019695, rpy=1.570796326794897 0 0
- Link `wrist_2_link`: mass=2.0869, inertia={ ixx=0.004288, ixy=1.7e-05, ixz=-3e-06, iyy=0.002566, iyz=-0.000716, izz=0.003927 }
  - 惯性原点：xyz=2.5e-05 0.015886 -0.01996, rpy=-1.570796326794897 0 0
- Link `wrist_3_link`: mass=1.0666, inertia={ ixx=0.001406, ixy=-3e-06, ixz=1e-05, iyy=0.001404, iyz=-2.4e-05, izz=0.001002 }
  - 惯性原点：xyz=-1.8e-05 -0.000112 -0.053397, rpy=0 0 0
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
- Joint `shoulder_pan_joint` limits: { lower=-6.28319, upper=6.28319, effort=433, velocity=3.14159 }
- Joint `shoulder_lift_joint` limits: { lower=-6.28319, upper=6.28319, effort=433, velocity=3.14159 }
- Joint `elbow_joint` limits: { lower=-3.14159, upper=3.14159, effort=204, velocity=4.18879 }
- Joint `wrist_1_joint` limits: { lower=-6.28319, upper=6.28319, effort=70, velocity=5.23599 }
- Joint `wrist_2_joint` limits: { lower=-6.28319, upper=6.28319, effort=70, velocity=5.23599 }
- Joint `wrist_3_joint` limits: { lower=-6.28319, upper=6.28319, effort=70, velocity=5.23599 }

### 4) ControlSubmodel
- Transmission `shoulder_pan_trans`: type=transmission_interface/SimpleTransmission, joints=['shoulder_pan_joint'], actuators=['shoulder_pan_motor']
- Transmission `shoulder_lift_trans`: type=transmission_interface/SimpleTransmission, joints=['shoulder_lift_joint'], actuators=['shoulder_lift_motor']
- Transmission `elbow_trans`: type=transmission_interface/SimpleTransmission, joints=['elbow_joint'], actuators=['elbow_motor']
- Transmission `wrist_1_trans`: type=transmission_interface/SimpleTransmission, joints=['wrist_1_joint'], actuators=['wrist_1_motor']
- Transmission `wrist_2_trans`: type=transmission_interface/SimpleTransmission, joints=['wrist_2_joint'], actuators=['wrist_2_motor']
- Transmission `wrist_3_trans`: type=transmission_interface/SimpleTransmission, joints=['wrist_3_joint'], actuators=['wrist_3_motor']

### 5) VisualizationSubmodel
- Link `base_link_inertia` → Visual: package://ur_description/meshes/ur15/visual/base.dae | Collision: package://ur_description/meshes/ur15/collision/base.stl | Materials: LightGrey
- Link `shoulder_link` → Visual: package://ur_description/meshes/ur15/visual/shoulder.dae | Collision: package://ur_description/meshes/ur15/collision/shoulder.stl | Materials: LightGrey
- Link `upper_arm_link` → Visual: package://ur_description/meshes/ur15/visual/upperarm.dae | Collision: package://ur_description/meshes/ur15/collision/upperarm.stl | Materials: LightGrey
- Link `forearm_link` → Visual: package://ur_description/meshes/ur15/visual/forearm.dae | Collision: package://ur_description/meshes/ur15/collision/forearm.stl | Materials: LightGrey
- Link `wrist_1_link` → Visual: package://ur_description/meshes/ur15/visual/wrist1.dae | Collision: package://ur_description/meshes/ur15/collision/wrist1.stl | Materials: LightGrey
- Link `wrist_2_link` → Visual: package://ur_description/meshes/ur15/visual/wrist2.dae | Collision: package://ur_description/meshes/ur15/collision/wrist2.stl | Materials: LightGrey
- Link `wrist_3_link` → Visual: package://ur_description/meshes/ur15/visual/wrist3.dae | Collision: package://ur_description/meshes/ur15/collision/wrist3.stl | Materials: LightGrey

## ConceptDescription（建议词条草案）
- jointLowerLimit / jointUpperLimit / jointMaxVelocity / jointMaxEffort
- linkMass / linkInertiaIxx/Iyy/Izz/Ixy/Ixz/Iyz
- visualMeshUri / collisionMeshUri / materialName
- transmissionType / actuatorName / transmissionJointRef
