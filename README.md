# 🤖 7-DOF Franka Panda ROS 2 Workspace

A **ROS 2-based motion planning and simulation system** powered by the **Franka Emika Panda robotic arm**. This project focuses on the integration of **MoveIt 2 motion planning**, **ros2_control**, and **Gazebo simulation** to create a robust framework for autonomous robotic manipulation.

---

## ✨ Features

- 🦾 **7-DOF Control**: Complete trajectory planning and execution for the Franka Panda arm.
- 🎯 **Motion Planning**: Advanced path planning and collision avoidance using **MoveIt 2**.
- 📊 **Visual Feedback**: Real-time monitoring and visualization via **RViz 2**.
- 🛠️ **Gazebo Integration**: Physics-based simulation environment for testing control algorithms.
- 🐍 **Python API**: High-level robot control using **PyMoveIt2** for rapid prototyping.
- 🐟 **FishROS Ready**: Optimized for one-click installation and environment setup.

---

## 🎯 What You'll Learn

- Setting up a complete ROS 2 robotic workspace for Franka Panda.
- Configuring **MoveIt 2** for 7-DOF robotic manipulators.
- Implementing motion planning and execution via Python scripts.
- Using **ros2_control** for hardware abstraction and simulation.
- Building modular ROS 2 packages for robotics research.

---

## 📋 Table of Contents

1. [🐟 Quick Installation (FishROS)](#-quick-installation-fishros)
2. [💻 Manual Environment Setup](#-manual-environment-setup)
3. [🏗️ Workspace Build](#️-workspace-build)
4. [🎮 Running the Project](#-running-the-project)
5. [⚠️ Troubleshooting](#️-troubleshooting)
6. [📚 References](#-references)

---

# 🐟 Quick Installation (FishROS)

For users on Ubuntu 22.04, the FishROS one-click tool is the most efficient way to set up the environment.

## Step 1: Install ROS 2 Humble

```bash
wget http://fishros.com/install -O fishros && . fishros
```

Recommended choices:

1.Select humble version.

2.Select Desktop version.

## Step 2: Install MoveIt 2 & ROS Controllers

Install the core robotic stacks required for planning and simulation:

```bash
sudo apt update && sudo apt install -y \
  ros-humble-moveit \
  ros-humble-moveit-ros-move-group \
  ros-humble-moveit-ros-planning-interface \
  ros-humble-moveit-visual-tools \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros-gz \
  ros-humble-joint-state-publisher-gui
```

## 📥 Step 3: Python Environment Setup

The workspace relies on specific Python libraries for mathematical transformations and high-level control API logic:

```bash
# Update pip first
pip3 install --upgrade pip

# Install required libraries
pip3 install --no-cache-dir \
  numpy==1.24.4 \
  transforms3d \
  scipy \
  pyyaml
```

# 🏗️ Workspace Build

## 1. Create and Clone

```bash
mkdir -p ~/7-DOF-panda_ws/src
cd ~/7-DOF-panda_ws/src
git clone https://github.com/Ziozz/7-DOF-panda_ws.git .
```

## 2. Install Package DependenciesUse rosdep to fetch any remaining system-level dependencies:

```bash
cd ~/7-DOF-panda_ws
sudo rosdep init # Run if you haven't initialized rosdep before
rosdep update
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
```

## 3. Build and SourceBash# Build the workspace

```bash
colcon build --symlink-install
# Source the environment
source install/setup.bash
# (Optional) Add to bashrc
echo "source ~/7-DOF-panda_ws/install/setup.bash" >> ~/.bashrc
```


# 🎮 Running the Project

## 1. Launch MoveIt 2 (RViz Visualization)To test motion planning in a virtual environment without physics:

```bash
ros2 launch panda_moveit_config demo.launch.py
```

## 2. Launch Gazebo SimulationTo start the robot in a world with gravity and collision physics:

```bash
ros2 launch panda_bringup pick_and_place.launch.py 
```

## 3. Run Custom Motion ScriptExecute your Python control logic (ensure you have sourced the workspace in the terminal):

```bash
ros2 run panda_pick_place pick_place_node
```

# ⚠️ Troubleshooting

## 🔴 Gazebo Crashes or Black ScreenThis is often related to GPU drivers. Try forcing software rendering:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch panda_bringup pick_and_place.launch.py 
```

# 🔴 Controller Manager Not Found

## Ensure you have installed the ros2_control suite:

```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
```

# 📚 References

- 🌐 [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- 🌐 [MoveIt 2 Tutorials](https://moveit.picknik.ai/humble/index.html)
- 🌐 [FishROS Official Website](https://fishros.org.cn/forum/)
- 🔗 [Inspired by MechaMind-Labs](https://github.com/MechaMind-Labs/Franka_Panda_Color_Sorting_Robot/tree/main)
