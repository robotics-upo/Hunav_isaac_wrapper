# **HuNav Isaac Wrapper**

A standalone simulation wrapper built on **NVIDIA Isaac Sim** that manages [HuNavSim](https://github.com/robotics-upo/hunav_sim) agents. It enables seamless integration of HuNavSim into Isaac Sim environments, supporting physics-based animations and ROS 2 teleoperation.

---

### 🚧 **Work in Progress**
This repository is under active development and subject to improvements.

### ✅ **Tested Configurations**
- **ROS 2 Humble**
- **Isaac Sim 4.2**
- **Ubuntu 22.04 LTS**

---

## 🔹 **Overview**
**HuNav Isaac Wrapper** provides a **modular** architecture for simulating **human navigation** in **NVIDIA Isaac Sim**, integrating **ROS 2** for teleoperation and **HuNavSim** for agent behavior simulation. Key components include:

- **ROS 2 Integration:**  
  `teleop_hunav_sim.py` implements the `TeleopHuNavSim` class, which subscribes to `/cmd_vel` for robot teleoperation and integrates **HuNavSim** within **Isaac Sim**.

- **Agent Management:**  
`hunav_manager.py` manages agent **spawning/management**, **physics**, and **obstacle detection** based on YAML configurations (`config/` folder).
  
- **Animation Retargeting:**  
  Supports **animation retargeting**, using **USD SkelAnimation** and the **Omni Anim Retargeting extension** (`CreateRetargetAnimationsCommand`) to apply a single set of animation clips (e.g., walking, idle) originally designed for a **default NVIDIA biped skeleton** to multiple character models.

- **World Loading:**  
  `world_builder.py` loads USD scenarios from the `scenarios/` folder into Isaac Sim.

---

## 🔹 **Features**

- **Modular and Extensible Architecture:**  
  Organized into independent modules for easy maintenance and expansion:
    - `world_builder.py` — Loads USD scenarios.
    - `hunav_manager.py` — Manages agent spawning and management, physics, and animation retargeting.
    - `teleop_hunav_sim.py` — Integrates ROS 2 teleoperation and initializes HuNavSim's workflow.
    - `main.py` — Entry point for the simulation.

- **Unified Animation Workflow:**  
  Retargets animations from a **default biped skeleton** to various character models using skeletal binding.

- **Configurable Agent Spawning:**  
  YAML configuration files (`config/`) define **initial positions, waypoints, social force model (SFM) parameters, and behaviors**.

- **Social simulation and Teleoperation:**  
  - **ROS 2 teleoperation** via `/cmd_vel` for real-time robot control.
  - **HuNavSim agent management** for social-aware human navigation.

- **Obstacle Detection:**  
  **PhysX raycasts** (implemented in `hunav_manager.py`) detect nearby obstacles for **HuNavSim’s** navigation logic.

---

## 🔹 **Requirements**
- **Ubuntu 22.04 LTS**
- [**HuNavSim**](https://github.com/robotics-upo/hunav_sim)
- [**NVIDIA Isaac Sim**](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html#): (Tested on version 4.2.0).  

  
- **Python 3.8+**  

- **ROS 2 (**only tested on [ROS Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)**)**  


---

## 🔹 **Installation and Usage**

### 1. Clone the Repository

```bash
git clone https://github.com/robotics-upo/Hunav_isaac_wrapper.git
cd Hunav_isaac_wrapper
```

### 2. Configure Your Environment

- Launch Isaac Sim’s Python interpreter or set up the environment so that all Omniverse APIs are available.
- **The provided startup bash script (`startup_hunav_isaac.sh`) should automatically load the correct environment.**


### 3. ROS 2 Setup

Ensure that ROS 2 is installed and sourced before launching the simulation.

### 4. Configure Your Agents and Scene

The map name and agent configuration file are set in `main.py`. 

#### Agents Configuration
- Edit the YAML file in the `config/` folder (e.g., `agents_1.yaml`) to specify agent parameters such as:
  - **Initial pose**: Set the starting position.
  - **Goals**: Define the waypoints or destinations for each agent.
  - **SFM weights**: Configure social force model parameters.
  - **Behavior**: Set the behavior type and associated parameters.

#### Scene Setup
- Place your USD scenario file (e.g., `empty_world.usd`) in the `scenarios/` folder.

### 5. Launch the Simulation

Run the startup script:

```bash
./startup_hunav_isaac.sh
```

This script will:
- Launch Isaac Sim (with a GUI unless headless mode is specified).
- Load the specified scenario.
- Spawn agents based on the YAML configuration.
- Apply physics, animations (including retargeting), and initialize ROS 2 communication for enabling HuNavSim and robot teleoperation.

### 6. Teleoperation and Simulation

#### Teleoperation
- Publish ROS 2 Twist messages to `/cmd_vel` to control the robot (e.g., a Jetbot).

#### Simulation
- The simulation continuously updates agent positions and behaviors at every physics step.


---

## 🔹 **Troubleshooting**

**(*) In the case of using agent models other than the provided ones**
### Agent Orientation (*)
- If agents appear horizontal rather than vertical, adjust the rotation applied in the script (e.g., modify the quaternion calculation in `hunav_manager.initialize_agents()`'s `init_rot` parameter and/or `hunav_manager._update_agents()`).

### Retargeting Errors (*)
- Verify that the default source biped prim is correctly loaded at `/World/biped_demo`.
- Ensure that the target agents have a valid skeleton (use the `findSkeletonPath` method for debugging).
- Confirm that the extension `omni.anim.retarget.core` is enabled.

### ROS 2 Connectivity
- Make sure ROS 2 is running and that the `/compute_agents` service is active.

---



