# URCell File Structure (Custom Depth Overview)

```
URCell/
├─ projects
│   ├─ cellA
│   │   ├─ igus_rebel_spawn.xacro
│   │   └─ ur5_spawn.xacro
│   └─ system
│       ├─ config
│       ├─ launch
│       ├─ multi_ur.xacro
│       ├─ rviz
│       └─ urdf
├─ tools
│   ├─ README.txt
│   ├─ add_kind_template.py
│   ├─ fix_aas_json.py
│   ├─ generate_aas_environment_rebel_v3.py
│   ├─ generate_aas_submodels_rebel.py
│   ├─ generate_aas_submodels_ur.py
│   ├─ generate_structure_doc.py
│   ├─ patch_aas_shells.py
│   ├─ patch_env_json.py
│   ├─ set_kind_and_semantic.py
│   └─ validate.py
├─ types
│   ├─ UR_types.aasx
│   ├─ backup
│   │   └─ backup007.aasx
│   ├─ igus_rebel_description_ros2
│   │   ├─ CMakeLists.txt
│   │   ├─ README.md
│   │   ├─ config
│   │   ├─ igus_rebel.urdf
│   │   ├─ launch
│   │   ├─ meshes
│   │   ├─ package.xml
│   │   ├─ rviz
│   │   └─ urdf
│   ├─ submodel
│   │   ├─ igus_rebel
│   │   │   └─ igus_rebel_environment.json
│   │   ├─ ur10
│   │   │   ├─ ur10_control_safe.json
│   │   │   ├─ ur10_dynamics_safe.json
│   │   │   ├─ ur10_environment.json
│   │   │   ├─ ur10_kinematics_safe.json
│   │   │   ├─ ur10_safety_safe.json
│   │   │   ├─ ur10_structure_safe_full.json
│   │   │   └─ ur10_visualization_safe.json
│   │   ├─ ur10e
│   │   │   ├─ ur10e_control_safe.json
│   │   │   ├─ ur10e_dynamics_safe.json
│   │   │   ├─ ur10e_environment.json
│   │   │   ├─ ur10e_kinematics_safe.json
│   │   │   ├─ ur10e_safety_safe.json
│   │   │   ├─ ur10e_structure_safe_full.json
│   │   │   └─ ur10e_visualization_safe.json
│   │   ├─ ur12e
│   │   │   ├─ ur12e_control_safe.json
│   │   │   ├─ ur12e_dynamics_safe.json
│   │   │   ├─ ur12e_environment.json
│   │   │   ├─ ur12e_kinematics_safe.json
│   │   │   ├─ ur12e_safety_safe.json
│   │   │   ├─ ur12e_structure_safe_full.json
│   │   │   └─ ur12e_visualization_safe.json
│   │   ├─ ur15
│   │   │   ├─ ur15_control_safe.json
│   │   │   ├─ ur15_dynamics_safe.json
│   │   │   ├─ ur15_environment.json
│   │   │   ├─ ur15_kinematics_safe.json
│   │   │   ├─ ur15_safety_safe.json
│   │   │   ├─ ur15_structure_safe_full.json
│   │   │   └─ ur15_visualization_safe.json
│   │   ├─ ur16e
│   │   │   ├─ ur16e_control_safe.json
│   │   │   ├─ ur16e_dynamics_safe.json
│   │   │   ├─ ur16e_environment.json
│   │   │   ├─ ur16e_kinematics_safe.json
│   │   │   ├─ ur16e_safety_safe.json
│   │   │   ├─ ur16e_structure_safe_full.json
│   │   │   └─ ur16e_visualization_safe.json
│   │   ├─ ur20
│   │   │   ├─ ur20_control_safe.json
│   │   │   ├─ ur20_dynamics_safe.json
│   │   │   ├─ ur20_environment.json
│   │   │   ├─ ur20_kinematics_safe.json
│   │   │   ├─ ur20_safety_safe.json
│   │   │   ├─ ur20_structure_safe_full.json
│   │   │   └─ ur20_visualization_safe.json
│   │   ├─ ur3
│   │   │   ├─ ur3_control.json
│   │   │   ├─ ur3_dynamics.json
│   │   │   ├─ ur3_environment.json
│   │   │   ├─ ur3_kinematics.json
│   │   │   ├─ ur3_safety.json
│   │   │   ├─ ur3_structure.json
│   │   │   └─ ur3_visualization.json
│   │   ├─ ur30
│   │   │   ├─ ur30_control_safe.json
│   │   │   ├─ ur30_dynamics_safe.json
│   │   │   ├─ ur30_environment.json
│   │   │   ├─ ur30_kinematics_safe.json
│   │   │   ├─ ur30_safety_safe.json
│   │   │   ├─ ur30_structure_safe_full.json
│   │   │   └─ ur30_visualization_safe.json
│   │   ├─ ur3e
│   │   │   ├─ ur3e_control_safe_prefixed.json
│   │   │   ├─ ur3e_dynamics_safe_prefixed.json
│   │   │   ├─ ur3e_environment_prefixed.json
│   │   │   ├─ ur3e_kinematics_safe_prefixed.json
│   │   │   ├─ ur3e_safety_safe_prefixed.json
│   │   │   ├─ ur3e_structure_safe_full_prefixed.json
│   │   │   └─ ur3e_visualization_safe_prefixed.json
│   │   ├─ ur5
│   │   │   ├─ ur5_control_safe_prefixed.json
│   │   │   ├─ ur5_dynamics_safe_prefixed.json
│   │   │   ├─ ur5_environment_prefixed.json
│   │   │   ├─ ur5_kinematics_safe_prefixed.json
│   │   │   ├─ ur5_safety_safe_prefixed.json
│   │   │   ├─ ur5_structure_safe_full_prefixed.json
│   │   │   └─ ur5_visualization_safe_prefixed.json
│   │   ├─ ur5e
│   │   │   ├─ ur5e_control_safe.json
│   │   │   ├─ ur5e_dynamics_safe.json
│   │   │   ├─ ur5e_environment.json
│   │   │   ├─ ur5e_kinematics_safe.json
│   │   │   ├─ ur5e_safety_safe.json
│   │   │   ├─ ur5e_structure_safe_full.json
│   │   │   └─ ur5e_visualization_safe.json
│   │   └─ ur7e
│   │       ├─ ur7e_control_safe.json
│   │       ├─ ur7e_dynamics_safe.json
│   │       ├─ ur7e_environment.json
│   │       ├─ ur7e_kinematics_safe.json
│   │       ├─ ur7e_safety_safe.json
│   │       ├─ ur7e_structure_safe_full.json
│   │       └─ ur7e_visualization_safe.json
│   ├─ ur_description
│   │   ├─ config
│   │   ├─ doc
│   │   ├─ launch
│   │   ├─ meshes
│   │   ├─ rviz
│   │   ├─ test
│   │   └─ urdf
│   │       ├─ inc
│   │       │   ├─ ur_common.xacro
│   │       │   └─ ur_transmissions.xacro
│   │       ├─ pythonProject
│   │       │   ├─ fix_urdf_paths.py
│   │       │   ├─ generate_aas_submodels.py
│   │       │   ├─ meshes
│   │       │   │   ├─ ur10
│   │       │   │   ├─ ur10e
│   │       │   │   ├─ ur15
│   │       │   │   ├─ ur16e
│   │       │   │   ├─ ur20
│   │       │   │   ├─ ur3
│   │       │   │   ├─ ur30
│   │       │   │   ├─ ur3e
│   │       │   │   ├─ ur5
│   │       │   │   └─ ur5e
│   │       │   ├─ ur10.urdf
│   │       │   ├─ ur10e.urdf
│   │       │   ├─ ur12e.urdf
│   │       │   ├─ ur15.urdf
│   │       │   ├─ ur16e.urdf
│   │       │   ├─ ur20.urdf
│   │       │   ├─ ur3.urdf
│   │       │   ├─ ur3.urdf.bak
│   │       │   ├─ ur30.urdf
│   │       │   ├─ ur3e.urdf
│   │       │   ├─ ur5.urdf
│   │       │   ├─ ur5e.urdf
│   │       │   ├─ ur7e.urdf
│   │       │   └─ urdfenv
│   │       │       ├─ Include
│   │       │       ├─ Lib
│   │       │       ├─ Scripts
│   │       │       ├─ pyvenv.cfg
│   │       │       └─ share
│   │       ├─ spawn_ur5.xacro
│   │       ├─ ur.ros2_control.xacro
│   │       ├─ ur.urdf.xacro
│   │       ├─ ur10.urdf
│   │       ├─ ur10e.urdf
│   │       ├─ ur12e.urdf
│   │       ├─ ur15.urdf
│   │       ├─ ur16e.urdf
│   │       ├─ ur20.urdf
│   │       ├─ ur3.urdf
│   │       ├─ ur30.urdf
│   │       ├─ ur3e.urdf
│   │       ├─ ur5.urdf
│   │       ├─ ur5e.urdf
│   │       ├─ ur7e.urdf
│   │       └─ ur_macro.xacro
│   └─ ur_robot_driver
│       ├─ CHANGELOG.rst
│       ├─ CMakeLists.txt
│       ├─ LICENSE
│       ├─ README.md
│       ├─ config
│       ├─ doc
│       ├─ hardware_interface_plugin.xml
│       ├─ include
│       ├─ launch
│       ├─ package.xml
│       ├─ resources
│       ├─ scripts
│       ├─ src
│       └─ test
└─ urcell_structure.md
```
