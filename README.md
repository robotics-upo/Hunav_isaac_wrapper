# **HuNav Isaac Wrapper**  

A standalone simulation wrapper for **NVIDIA Isaac Sim**, integrating the **Human Navigation Simulator ([HuNavSim](https://github.com/robotics-upo/hunav_sim))** with **physics-based animations** and **ROS 2 integration**.

---

### 🚧 **Work in Progress**

This repository is actively developed and subject to improvements.

### ✅ **Tested Configurations**  

- **ROS 2 Humble**  
- **Isaac Sim 4.2**  
- **Ubuntu 22.04 LTS**  

---

## 🔹 **Overview**  

**HuNav Isaac Wrapper** is a modular simulation framework that integrates the [HuNavSim](https://github.com/robotics-upo/hunav_sim) human navigation framework into **NVIDIA Isaac Sim**, enabling realistic multi-agent behavior with physics-based animation and **ROS 2** interoperability.

It supports both **ROS 2 teleoperation** and **autonomous navigation (Nav2)**, scenario loading, dynamic agent configuration, and multiple robot models.

> This wrapper is built for research in **human-robot interaction**, **social navigation**, and **simulation-based validation of social navigation policies**.

---

## 🔹 Features  

- **Modular Architecture:**  
  - `main.py`: Launches simulation entry point.
  - `world_builder.py`: Loads USD scenario files.
  - `hunav_manager.py`: Handles agent creation, communication with HuNavSim services, and manages physics, animations, and obstacle detection.
  - `teleop_hunav_sim.py`: Manages HuNavSim initialization, updates agent states, and handles the ROS 2 /cmd_vel interface for robot control.
  - `animation_utils.py`: Utilities for AnimationGraph setup and retargeting.

- **Animation System:**  
  - **AnimationGraph-based** blending for smooth walk/idle transitions.  
    - Driven by agent velocity, allowing dynamic switching between walk and idle states.
  - Supports animation **retargeting**, applying a single set of animations to different characters via **USD SkelAnimation** and the **Omni Anim Retargeting extension**.

- **Flexible Agent Configuration:**  
  - YAML files (in `config/`) define agent spawn positions, navigation goals, SFM parameters, and behavior profiles for each scenario.

- **Multiple Robot Models:**  
  - Includes `jetbot`, `create3`, `carter`, and `carter_ROS` models.

- **ROS 2 Navigation (Nav2) Support:**  
  - Enables autonomous navigation for the **Carter** robot using the **ROS 2 Nav2** stack.

- **Social Simulation & Teleoperation:**  
  - Real-time robot control via `/cmd_vel`.  
  - Socially-aware agent movement via **HuNavSim** integration.

- **Obstacle Detection:**  
  - Uses **PhysX raycasts** (in `hunav_manager.py`) for detecting obstacles and informing **HuNavSim** navigation logic.

---

## 🔹 **Requirements**

- **Ubuntu 22.04 LTS**
- [**HuNavSim**](https://github.com/robotics-upo/hunav_sim)
- [**NVIDIA Isaac Sim**](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_workstation.html#) (Tested on version 4.2.0)

- **Python 3.8+**  

- **ROS 2 **[Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)****  

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

> If needed, follow this guide ([ROS2 Installation](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html)) to make sure the ROS 2 workspace environment is setup correctly.

### 4. Configure Your Scene, Agents, and Robot

The simulation setup is initialized in `main.py`, where you can define the scenario, agent configuration, and robot model to use.

#### 🗺️ Scene Setup

The repository includes multiple pre-configured scenario USD files located in the `scenarios/` folder, each reflecting a different type of environment:

- `warehouse.usd`: Industrial layout with shelves and various obstacles.
- `hospital.usd`: Medical environment with corridors and rooms.
- `office.usd`: Workspace with desks and conference areas.
- `empty_world.usd`: A minimal open environment for testing.

#### 🧍 Agents Configuration

Each scenario is paired with a YAML file in the `config/` folder that defines the HuNavSim agents to be used within it.

- **Available configuration files:**
  - `agents_warehouse.yaml` → for `warehouse.usd`
  - `agents_hospital.yaml` → for `hospital.usd`

- **Each YAML file lets you configure:**
  - **Initial pose**: Define starting positions of each agent.
  - **Goals**: Set destination coordinates or waypoints per agent.
  - **SFM weights**: Tune the social force model for realistic crowd behavior.
  - **Behavior type**: Choose how agents behave.

> Always pair the scenario with its corresponding agent YAML to avoid misaligned goals or initial agent positions.

#### 🤖 Robot Configuration

- Select your desired robot from the available options in `main.py`:
  - `jetbot`, `create3`, `carter`, or `carter_ROS`
  > **Note:** For `carter_ROS`, make sure to unzip the `nova_carter_ros2_sensors` package located in `config/robots/`.
- **Carter** robot also supports **ROS 2 Navigation (Nav2)** for autonomous navigation (see *Teleoperation and Simulation* section for launch instructions).

### 5. Launch the Simulation

Run the startup script:

```bash
./startup_hunav_isaac.sh
```

This script will:

- Launch Isaac Sim (with a GUI unless headless mode is specified).
- Load the specified scenario.
- Spawn agents based on the YAML configuration.
- Apply physics, animations, and initialize integration with ROS 2 and HuNavSim.

### 6. Teleoperation and Simulation

#### Teleoperation

- Use ROS 2 to publish Twist messages to `/cmd_vel` for direct robot control during the simulation.

#### Simulation

- Agent states (position, behavior, animation) are updated every physics step.
- The robot can be teleoperated in real-time, and agents navigate according to their goals and behaviors.

#### ROS 2 Navigation (Nav2) – Carter Robot Only

If you're using the `carter_ROS` robot model and want to enable autonomous navigation:

1. Make sure the simulation is running (`startup_hunav_isaac.sh` launched successfully).
2. In a separate terminal (with your ROS 2 environment sourced), launch the navigation stack:

   ```bash
   cd Hunav_isaac_wrapper
   ros2 launch carter_navigation carter_navigation.launch.py params_file:="config/navigation_params/carter_navigation_params.yaml"
   ```

---

## 🔹 **Troubleshooting**

**(*) In the case of using agent models other than the provided ones**

### ROS 2 Connectivity

- Make sure that your existing ROS 2 Humble installation is sourced

    ```bash
  source /opt/ros/humble/setup.bash 
  ```

- If ``carter_navigation`` package is not recognized, follow these steps:

  Clone the [IsaacSim-ros_workspaces](https://github.com/isaac-sim/IsaacSim-ros_workspaces.git) repository

    ```bash
  git clone https://github.com/isaac-sim/IsaacSim-ros_workspaces.git
  ```

  Build the ROS 2 humble workspace

    ```bash
  cd IsaacSim-ros_workspaces/humble_ws
  colcon build
  ```

  Then make sure to source the workspace inside your `.bashrc`

    ```bash
  source ~/IsaacSim-ros_workspaces/humble_ws/install/setup.bash
  ```

### Agent Orientation (*)

- If agents appear horizontal rather than vertical, adjust the rotation applied in the script (e.g., modify the quaternion calculation in `hunav_manager.initialize_agents()`'s `init_rot` parameter and/or `hunav_manager._update_agents()`).

### Retargeting Errors (*)

- Verify that the default source biped prim is correctly loaded at `/World/biped_demo`.
- Ensure that the target agents have a valid **skeleton** (use the `findSkeletonPath` method for debugging).
- Confirm that the extension `omni.anim.retarget.core` is enabled.

### AnimationGraph Issues

- If AnimationGraph is not applying correctly, verify that it is correctly assigned to the agent's **SkelRoot** and that transformations are applied properly.

---
