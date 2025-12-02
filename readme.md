# URCell — Digital Twin Infrastructure for Industrial Robots  
### URDF → AAS Transformation | Multi-Robot Modeling | Industrial Digital Twin Generation

URCell is the complete software artefact developed for the Bachelor's thesis  
**"Design and Implementation of a Digital Twin Infrastructure for Industrial Production"**  
at **Karlsruhe Institute of Technology (KIT)**.

URCell implements an extensible digital twin pipeline that transforms **URDF** robot descriptions into structured **Asset Administration Shell (AAS)** Submodels and generates **multi-robot industrial environments**.  
It serves as a unified digital twin infrastructure for industrial robotics applications.

---

# 📌 Key Contributions

URCell provides:

## ✔ 1. URDF → AAS Transformation Pipeline  
Automatically extracts semantic information from URDF files and generates AAS-compliant Submodels:
- Structure Submodel  
- Control Submodel  
- Kinematics Submodel  
- Dynamics Submodel  
- Safety Submodel  
- Visualization Submodel  

## ✔ 2. Environment Model Generator  
Creates robot-specific `environment.json` files used in assembling complete AAS models.

## ✔ 3. Multi-Robot Factory Cell Generator  
Generates Xacro/URDF multi-robot environments from YAML configuration files:
- Multiple UR robots (UR3 / UR5 / UR10)
- Mixed-brand setups (e.g., igus Rebel)
- Automatic placement and naming

## ✔ 4. Reproducible Digital Twin Artefacts  
URCell outputs all artefacts needed for AAS modelling and digital twin simulation:
- Submodel JSON files  
- Xacro files (`*_spawn.xacro`)  
- `multi_ur.xacro` multi-robot scene  
- Per-robot generated metadata  

---

# 📂 Project Structure

URCell/
│
├── src/ # Core processing pipeline
│ ├── urdf_parser/ # URDF semantic extraction
│ ├── aas_generator/ # Submodel generation modules
│ └── multi_robot/ # Multi-robot model builder
│
├── config/ # Robot & environment configuration files
│ ├── ur3/
│ ├── ur5/
│ ├── ur10/
│ └── igus_rebel/
│
├── types/ # Submodel templates / environment definitions
│ └── submodel/
│
├── tools/ # High-level generation scripts
│ ├── generate_submodels.py
│ ├── generate_environment.py
│ └── generate_multi_robot_xacro.py
│
├── projects/ # Generated Xacro files (spawns and multi-robot)
│
├── examples/ # Example URDFs, configs and outputs
│
└── README.md

---

# 🚀 Installation

### Requirements
- Python 3.10+
- `pip`
- Works on: Windows / Linux / macOS

Install dependencies:

```bash
pip install -r requirements.txt
🧪 Usage
1️⃣ Generate AAS Submodels from URDF
python tools/generate_submodels.py \
    --urdf examples/ur5/ur5.urdf \
    --output output/ur5/


Outputs include:

Structure.json

Kinematics.json

Dynamics.json

Control.json

Safety.json

Visualization.json

2️⃣ Generate Environment Config
python tools/generate_environment.py \
    --config config/ur5/ur5_config.yaml \
    --out output/ur5/environment.json

3️⃣ Generate Multi-Robot Workcell (URCell)
python tools/generate_multi_robot_xacro.py \
    --config config/multi_ur.yaml


Generated outputs:

projects/spawns/ur3_spawn.xacro

projects/spawns/ur5_spawn.xacro

projects/multi_ur.xacro

📘 Examples Included

URCell includes pre-configured examples for:

UR3

UR5

UR10

igus Rebel

Mixed multi-robot workcell

Each example can be fully regenerated using the provided scripts.

🧩 Final Submission Version

The exact artefact submitted with the thesis is archived as:

👉 v1.0-final
https://github.com/StUBenHe/URCell/releases/tag/v1.0-final

This version is permanently frozen and corresponds exactly to the results presented in the thesis.

📄 License

URCell is released under the MIT License.
See the LICENSE file for details.

🙋 Contact

Author: Ben He
Institution: Karlsruhe Institute of Technology (KIT)
For academic or industrial use, please reference or cite this repository when appropriate.