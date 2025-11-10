# URCell File Structure (Custom Depth Overview)

```
URCell/
├─ projects
│   ├─ multi_ur.xacro
│   └─ spawns
│       ├─ igus_rebel_spawn.xacro
│       └─ ur5_spawn.xacro
├─ tools
│   ├─ __pycache__
│   │   └─ generate_aas_submodels_ur.cpython-312.pyc
│   ├─ check_env.py
│   ├─ env_report.txt
│   ├─ generate_aas_submodels_igus_rebel.py
│   ├─ generate_aas_submodels_ur.py
│   ├─ generate_spawn_xacro.py
│   ├─ generate_structure_doc.py
│   ├─ output
│   │   ├─ debug_ur10_fixed.urdf
│   │   ├─ debug_ur10e_fixed.urdf
│   │   ├─ debug_ur12e_fixed.urdf
│   │   ├─ debug_ur15_fixed.urdf
│   │   ├─ debug_ur16e_fixed.urdf
│   │   ├─ debug_ur20_fixed.urdf
│   │   ├─ debug_ur30_fixed.urdf
│   │   ├─ debug_ur3_fixed.urdf
│   │   ├─ debug_ur3e_fixed.urdf
│   │   ├─ debug_ur5_fixed.urdf
│   │   ├─ debug_ur5e_fixed.urdf
│   │   └─ debug_ur7e_fixed.urdf
│   └─ probe program
│       ├─ add_kind_template.py
│       ├─ fix_aas_json.py
│       ├─ generate_aas_environment_ur.py
│       ├─ generate_aas_from_urdf.py
│       ├─ kind_semantic_tool.py
│       ├─ patch_aas_shells.py
│       ├─ patch_env_json.py
│       └─ set_kind_and_semantic.py
├─ types
│   ├─ Ropot_types.aasx
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
│   │   │   ├─ igus_rebel_6dof_control_submodel.json
│   │   │   ├─ igus_rebel_6dof_dynamics_submodel.json
│   │   │   ├─ igus_rebel_6dof_environment.json
│   │   │   ├─ igus_rebel_6dof_kinematics_submodel.json
│   │   │   ├─ igus_rebel_6dof_safety_submodel.json
│   │   │   ├─ igus_rebel_6dof_structure_submodel.json
│   │   │   └─ igus_rebel_6dof_visualization_submodel.json
│   │   ├─ ur10
│   │   │   ├─ ur10_control_submodel.json
│   │   │   ├─ ur10_dynamics_submodel.json
│   │   │   ├─ ur10_environment.json
│   │   │   ├─ ur10_kinematics_submodel.json
│   │   │   ├─ ur10_safety_submodel.json
│   │   │   ├─ ur10_structure_submodel.json
│   │   │   └─ ur10_visualization_submodel.json
│   │   ├─ ur10e
│   │   │   ├─ ur10e_control_submodel.json
│   │   │   ├─ ur10e_dynamics_submodel.json
│   │   │   ├─ ur10e_environment.json
│   │   │   ├─ ur10e_kinematics_submodel.json
│   │   │   ├─ ur10e_safety_submodel.json
│   │   │   ├─ ur10e_structure_submodel.json
│   │   │   └─ ur10e_visualization_submodel.json
│   │   ├─ ur12e
│   │   │   ├─ ur12e_control_submodel.json
│   │   │   ├─ ur12e_dynamics_submodel.json
│   │   │   ├─ ur12e_environment.json
│   │   │   ├─ ur12e_kinematics_submodel.json
│   │   │   ├─ ur12e_safety_submodel.json
│   │   │   ├─ ur12e_structure_submodel.json
│   │   │   └─ ur12e_visualization_submodel.json
│   │   ├─ ur15
│   │   │   ├─ ur15_control_submodel.json
│   │   │   ├─ ur15_dynamics_submodel.json
│   │   │   ├─ ur15_environment.json
│   │   │   ├─ ur15_kinematics_submodel.json
│   │   │   ├─ ur15_safety_submodel.json
│   │   │   ├─ ur15_structure_submodel.json
│   │   │   └─ ur15_visualization_submodel.json
│   │   ├─ ur16e
│   │   │   ├─ ur16e_control_submodel.json
│   │   │   ├─ ur16e_dynamics_submodel.json
│   │   │   ├─ ur16e_environment.json
│   │   │   ├─ ur16e_kinematics_submodel.json
│   │   │   ├─ ur16e_safety_submodel.json
│   │   │   ├─ ur16e_structure_submodel.json
│   │   │   └─ ur16e_visualization_submodel.json
│   │   ├─ ur20
│   │   │   ├─ ur20_control_submodel.json
│   │   │   ├─ ur20_dynamics_submodel.json
│   │   │   ├─ ur20_environment.json
│   │   │   ├─ ur20_kinematics_submodel.json
│   │   │   ├─ ur20_safety_submodel.json
│   │   │   ├─ ur20_structure_submodel.json
│   │   │   └─ ur20_visualization_submodel.json
│   │   ├─ ur3
│   │   │   ├─ ur3_control_submodel.json
│   │   │   ├─ ur3_dynamics_submodel.json
│   │   │   ├─ ur3_environment.json
│   │   │   ├─ ur3_kinematics_submodel.json
│   │   │   ├─ ur3_safety_submodel.json
│   │   │   ├─ ur3_structure_submodel.json
│   │   │   └─ ur3_visualization_submodel.json
│   │   ├─ ur30
│   │   │   ├─ ur30_control_submodel.json
│   │   │   ├─ ur30_dynamics_submodel.json
│   │   │   ├─ ur30_environment.json
│   │   │   ├─ ur30_kinematics_submodel.json
│   │   │   ├─ ur30_safety_submodel.json
│   │   │   ├─ ur30_structure_submodel.json
│   │   │   └─ ur30_visualization_submodel.json
│   │   ├─ ur3e
│   │   │   ├─ ur3e_control_submodel.json
│   │   │   ├─ ur3e_dynamics_submodel.json
│   │   │   ├─ ur3e_environment.json
│   │   │   ├─ ur3e_kinematics_submodel.json
│   │   │   ├─ ur3e_safety_submodel.json
│   │   │   ├─ ur3e_structure_submodel.json
│   │   │   └─ ur3e_visualization_submodel.json
│   │   ├─ ur5
│   │   │   ├─ ur5_control_submodel.json
│   │   │   ├─ ur5_dynamics_submodel.json
│   │   │   ├─ ur5_environment.json
│   │   │   ├─ ur5_kinematics_submodel.json
│   │   │   ├─ ur5_safety_submodel.json
│   │   │   ├─ ur5_structure_submodel.json
│   │   │   └─ ur5_visualization_submodel.json
│   │   ├─ ur5e
│   │   │   ├─ ur5e_control_submodel.json
│   │   │   ├─ ur5e_dynamics_submodel.json
│   │   │   ├─ ur5e_environment.json
│   │   │   ├─ ur5e_kinematics_submodel.json
│   │   │   ├─ ur5e_safety_submodel.json
│   │   │   ├─ ur5e_structure_submodel.json
│   │   │   └─ ur5e_visualization_submodel.json
│   │   └─ ur7e
│   │       ├─ ur7e_control_submodel.json
│   │       ├─ ur7e_dynamics_submodel.json
│   │       ├─ ur7e_environment.json
│   │       ├─ ur7e_kinematics_submodel.json
│   │       ├─ ur7e_safety_submodel.json
│   │       ├─ ur7e_structure_submodel.json
│   │       └─ ur7e_visualization_submodel.json
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
