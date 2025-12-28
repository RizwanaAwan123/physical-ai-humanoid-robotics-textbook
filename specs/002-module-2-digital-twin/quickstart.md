# Module 2: Digital Twin Simulation - Development Environment Quickstart

**Feature**: 002-module-2-digital-twin
**Created**: 2025-12-07
**Status**: Complete
**Purpose**: Guide for setting up development environment for Module 2 content generation and testing

---

## Overview

This quickstart guide provides step-by-step instructions for setting up the complete development environment required to generate, test, and validate Module 2 content (Digital Twin Simulation - Gazebo & Unity).

**Target Audience**: Content developers, QA engineers, specialized Claude Code Subagents

**Estimated Setup Time**: 2-3 hours (including downloads and installations)

---

## Table of Contents

1. [System Requirements](#1-system-requirements)
2. [Platform Selection](#2-platform-selection)
3. [ROS 2 Humble Installation](#3-ros-2-humble-installation)
4. [Gazebo Classic Installation](#4-gazebo-classic-installation)
5. [Unity Installation](#5-unity-installation)
6. [Python Environment Setup](#6-python-environment-setup)
7. [Project Structure Setup](#7-project-structure-setup)
8. [Validation and Testing](#8-validation-and-testing)
9. [Troubleshooting](#9-troubleshooting)

---

## 1. System Requirements

### 1.1 Reference Hardware

**Minimum Requirements**:
- **CPU**: Intel i5-8th gen / AMD Ryzen 5 3600 or equivalent (4 cores, 8 threads)
- **RAM**: 16 GB DDR4
- **GPU**: NVIDIA GTX 1060 6GB / AMD RX 580 8GB or equivalent
- **Storage**: 50 GB free SSD space
- **Internet**: Broadband connection (for package downloads)

**Recommended Requirements**:
- **CPU**: Intel i7-10th gen / AMD Ryzen 7 5800X or better
- **RAM**: 32 GB DDR4
- **GPU**: NVIDIA RTX 3060 / AMD RX 6700 XT or better
- **Storage**: 100 GB free NVMe SSD space

### 1.2 Operating System

**Primary Platform**: Ubuntu 22.04 LTS (Jammy Jellyfish)

**Alternative Platforms**:
- Ubuntu 24.04 LTS (Noble Numbat) - ROS 2 Jazzy required instead of Humble
- Windows 11 with WSL2 (Ubuntu 22.04) - Gazebo only; Unity runs natively
- macOS (Unity only; Gazebo not officially supported)

**Note**: All instructions below assume Ubuntu 22.04 LTS unless specified otherwise.

---

## 2. Platform Selection

### 2.1 Decision Matrix

Choose your development approach based on your operating system:

| Operating System | Gazebo Setup | Unity Setup | ROS 2 Setup |
|-----------------|--------------|-------------|-------------|
| **Linux (Ubuntu 22.04)** | Native install | Native install | Native install |
| **Windows 11** | WSL2 + Docker | Native install | WSL2 |
| **macOS** | ❌ Not supported | Native install | Docker |

### 2.2 Recommended Approach by Chapter

- **Chapter 1 (Gazebo Physics)**: Linux native or WSL2
- **Chapter 2 (Unity Rendering)**: Any platform (Unity cross-platform)
- **Chapter 3 (Sensor Simulation)**: Linux native (both Gazebo and Unity)
- **Chapter 4 (Integration)**: Linux native (ROS 2 bridge requires all components)

**Recommendation**: Use Ubuntu 22.04 LTS natively for full-stack development.

---

## 3. ROS 2 Humble Installation

### 3.1 Ubuntu 22.04 Native Installation

```bash
# 1. Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. Setup sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. Install ROS 2 Humble Desktop (includes RViz, demos, tutorials)
sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-desktop -y

# 4. Install development tools
sudo apt install ros-dev-tools -y
sudo apt install python3-colcon-common-extensions -y
sudo apt install python3-rosdep python3-vcstool -y

# 5. Initialize rosdep
sudo rosdep init
rosdep update

# 6. Setup environment (add to ~/.bashrc for persistence)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3.2 Verification

```bash
# Check ROS 2 installation
ros2 --version
# Expected output: ros2 cli version 0.18.x

# Test with demo nodes
ros2 run demo_nodes_cpp talker &
ros2 run demo_nodes_cpp listener
# Should see "I heard: [Hello World: X]" messages

# Clean up
pkill -f demo_nodes
```

### 3.3 Windows (WSL2) Installation

```powershell
# In PowerShell (as Administrator)
wsl --install -d Ubuntu-22.04
wsl --set-version Ubuntu-22.04 2
wsl --set-default Ubuntu-22.04

# Restart computer, then open Ubuntu-22.04 terminal
# Follow Section 3.1 instructions inside WSL2
```

**Important**: WSL2 GUI support (WSLg) is required for Gazebo GUI. Verify with:

```bash
echo $DISPLAY
# Should output: :0 or similar
```

---

## 4. Gazebo Classic Installation

### 4.1 Ubuntu 22.04 Native Installation

```bash
# 1. Install Gazebo Classic 11 with ROS 2 integration
sudo apt install ros-humble-gazebo-ros-pkgs -y
sudo apt install ros-humble-gazebo-plugins -y
sudo apt install gazebo -y

# 2. Install additional Gazebo packages
sudo apt install libgazebo11-dev -y

# 3. Verify installation
gazebo --version
# Expected output: Gazebo multi-robot simulator, version 11.14.0
```

### 4.2 Performance Optimization

```bash
# Enable GPU acceleration (NVIDIA GPUs)
# Check if NVIDIA drivers are installed
nvidia-smi

# If not installed, install NVIDIA drivers (example for RTX 3060)
sudo apt install nvidia-driver-525 -y
sudo reboot

# Verify GPU is available to Gazebo
export GAZEBO_IP=127.0.0.1
gazebo --verbose
# Should see "Render engine: ogre" in logs
```

### 4.3 WSL2 + Docker Installation (Windows)

```bash
# Inside WSL2 Ubuntu 22.04

# 1. Install Docker
sudo apt update
sudo apt install docker.io -y
sudo usermod -aG docker $USER
newgrp docker

# 2. Pull Gazebo Docker image
docker pull osrf/ros:humble-desktop

# 3. Run Gazebo in container with GUI support (WSLg)
docker run -it --rm \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device /dev/dri \
  osrf/ros:humble-desktop \
  gazebo
```

### 4.4 Verification

```bash
# Test 1: Launch Gazebo with empty world
gazebo --verbose

# Test 2: Launch Gazebo with ROS 2 integration
ros2 launch gazebo_ros gazebo.launch.py

# Test 3: Spawn a model
ros2 run gazebo_ros spawn_entity.py -entity box -database box
```

**Expected Results**:
- Gazebo GUI opens without errors
- Real-Time Factor (RTF) ≥ 0.9 (check bottom-right corner)
- Model spawns successfully

---

## 5. Unity Installation

### 5.1 Unity Hub Installation

**Linux (Ubuntu 22.04)**:

```bash
# 1. Download Unity Hub AppImage
wget https://public-cdn.cloud.unity3d.com/hub/prod/UnityHub.AppImage

# 2. Make executable and run
chmod +x UnityHub.AppImage
./UnityHub.AppImage
```

**Windows**:

Download Unity Hub from https://unity.com/download and run installer.

**macOS**:

Download Unity Hub from https://unity.com/download and drag to Applications.

### 5.2 Unity Editor Installation

1. Open Unity Hub
2. Navigate to **Installs** → **Install Editor**
3. Select **Unity 2022.3 LTS** (Long Term Support)
4. Add modules:
   - ✅ Linux Build Support (if on Linux)
   - ✅ Windows Build Support (Mono) (if on Windows)
   - ✅ Documentation
   - ✅ Language Pack (if needed)
5. Click **Install**

### 5.3 Unity Packages Installation

Create a new Unity project to install required packages:

1. Open Unity Hub → **Projects** → **New Project**
2. Select **3D (URP)** template
3. Name: `HumanoidDigitalTwin`
4. Create project
5. Once opened, go to **Window** → **Package Manager**
6. Install the following packages:

**Required Packages**:
- **URDF Importer** (com.unity.robotics.urdf-importer)
  - Search: "URDF Importer"
  - Version: 0.5.2 or later
- **ROS TCP Connector** (com.unity.robotics.ros-tcp-connector)
  - Search: "ROS TCP Connector"
  - Version: 0.7.0 or later
- **Perception** (com.unity.perception)
  - Search: "Perception"
  - Version: 1.0.0 or later

**Optional Packages**:
- **Unity ML-Agents** (com.unity.ml-agents)
  - Search: "ML-Agents"
  - Version: 3.0.0 or later

### 5.4 Verification

1. **URDF Importer Test**:
   - Download sample URDF: `wget https://raw.githubusercontent.com/ros/urdf_tutorial/ros2/urdf/01-myfirst.urdf`
   - In Unity: **Assets** → **Import Robot from URDF** → Select `01-myfirst.urdf`
   - Robot should appear in scene

2. **ROS TCP Connector Test**:
   - In Unity: **Robotics** → **ROS Settings**
   - Set **ROS IP Address**: `127.0.0.1`
   - Set **ROS Port**: `10000`
   - Click **Connect** (should show "Awaiting connection")

---

## 6. Python Environment Setup

### 6.1 Python Version Check

```bash
python3 --version
# Expected: Python 3.10.x or 3.11.x
```

### 6.2 Virtual Environment Setup

```bash
# 1. Install venv
sudo apt install python3-venv python3-pip -y

# 2. Create virtual environment for RL training
mkdir -p ~/humanoid_robotics/rl_training
cd ~/humanoid_robotics/rl_training
python3 -m venv venv

# 3. Activate virtual environment
source venv/bin/activate

# 4. Upgrade pip
pip install --upgrade pip setuptools wheel
```

### 6.3 Install Required Python Packages

```bash
# Activate virtual environment first
source ~/humanoid_robotics/rl_training/venv/bin/activate

# 1. Install stable-baselines3 (RL framework)
pip install stable-baselines3[extra]

# 2. Install Gymnasium (RL environment interface)
pip install gymnasium

# 3. Install ROS 2 Python dependencies
pip install rclpy sensor-msgs-py geometry-msgs-py

# 4. Install data processing libraries
pip install numpy pandas matplotlib

# 5. Install testing libraries
pip install pytest pytest-timeout

# 6. Save requirements
pip freeze > requirements.txt
```

### 6.4 Verification

```python
# Test stable-baselines3
python3 -c "import stable_baselines3; print(stable_baselines3.__version__)"
# Expected: 2.x.x

# Test Gymnasium
python3 -c "import gymnasium; env = gymnasium.make('CartPole-v1'); print('Gymnasium OK')"
# Expected: Gymnasium OK

# Test ROS 2 Python
python3 -c "import rclpy; print('rclpy OK')"
# Expected: rclpy OK
```

---

## 7. Project Structure Setup

### 7.1 Clone Repository

```bash
cd ~
git clone https://github.com/alishbanasir/hackathon1-humanoid-robotics-book.git
cd hackathon1-humanoid-robotics-book
```

### 7.2 Create Module 2 Directory Structure

```bash
# Create examples directories
mkdir -p examples/gazebo/{worlds,models,launch,config,tests}
mkdir -p examples/unity/{Scenes,Scripts,Prefabs,Materials}
mkdir -p examples/urdf
mkdir -p examples/ros2_ws/src
mkdir -p examples/reinforcement_learning/{environments,training,checkpoints}

# Create docs directories
mkdir -p docs/module-2/assets/{diagrams,screenshots,videos}

# Verify structure
tree -L 3 examples/
```

Expected output:

```
examples/
├── gazebo/
│   ├── config/
│   ├── launch/
│   ├── models/
│   ├── tests/
│   └── worlds/
├── reinforcement_learning/
│   ├── checkpoints/
│   ├── environments/
│   └── training/
├── ros2_ws/
│   └── src/
├── unity/
│   ├── Materials/
│   ├── Prefabs/
│   ├── Scenes/
│   └── Scripts/
└── urdf/
```

### 7.3 Initialize ROS 2 Workspace

```bash
cd examples/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

# Add to ~/.bashrc for convenience
echo "source ~/hackathon1-humanoid-robotics-book/examples/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## 8. Validation and Testing

### 8.1 End-to-End Validation Script

Create `validate_environment.sh`:

```bash
#!/bin/bash

echo "=== Module 2 Development Environment Validation ==="
echo ""

# 1. ROS 2 Check
echo "[1/7] Checking ROS 2 Humble..."
if ros2 --version | grep -q "0.18"; then
  echo "✅ ROS 2 Humble installed"
else
  echo "❌ ROS 2 Humble not found"
  exit 1
fi

# 2. Gazebo Check
echo "[2/7] Checking Gazebo Classic..."
if gazebo --version | grep -q "11.14"; then
  echo "✅ Gazebo Classic 11.14 installed"
else
  echo "❌ Gazebo Classic 11.14 not found"
  exit 1
fi

# 3. Gazebo ROS Packages Check
echo "[3/7] Checking Gazebo ROS packages..."
if ros2 pkg list | grep -q "gazebo_ros"; then
  echo "✅ gazebo_ros_pkgs installed"
else
  echo "❌ gazebo_ros_pkgs not found"
  exit 1
fi

# 4. Python Packages Check
echo "[4/7] Checking Python packages..."
python3 -c "import stable_baselines3, gymnasium, rclpy" 2>/dev/null
if [ $? -eq 0 ]; then
  echo "✅ Python packages (SB3, Gymnasium, rclpy) installed"
else
  echo "❌ Python packages missing"
  exit 1
fi

# 5. Unity Check (manual)
echo "[5/7] Checking Unity installation..."
if command -v unity-editor &> /dev/null || [ -f "./UnityHub.AppImage" ]; then
  echo "⚠️  Unity Hub found (manual verification required)"
else
  echo "❌ Unity Hub not found"
  exit 1
fi

# 6. GPU Check
echo "[6/7] Checking GPU acceleration..."
if command -v nvidia-smi &> /dev/null; then
  echo "✅ NVIDIA GPU detected"
  nvidia-smi --query-gpu=name --format=csv,noheader
else
  echo "⚠️  NVIDIA GPU not detected (acceptable for Unity-only development)"
fi

# 7. Workspace Check
echo "[7/7] Checking ROS 2 workspace..."
if [ -f "examples/ros2_ws/install/setup.bash" ]; then
  echo "✅ ROS 2 workspace initialized"
else
  echo "❌ ROS 2 workspace not built"
  exit 1
fi

echo ""
echo "=== Validation Complete ==="
echo "✅ Environment is ready for Module 2 development"
```

Run validation:

```bash
chmod +x validate_environment.sh
./validate_environment.sh
```

### 8.2 Performance Benchmarks

**Gazebo RTF Benchmark**:

```bash
# Launch Gazebo with empty world and measure RTF
timeout 60s gazebo --verbose 2>&1 | grep "Factor"
```

Expected output: `Factor[ 1.0x ]` or higher

**Unity FPS Benchmark** (manual):

1. Open Unity project
2. Open any scene
3. Press **Play**
4. Check **Stats** overlay (top-right corner)
5. Verify FPS ≥ 60

---

## 9. Troubleshooting

### 9.1 Common Issues

#### Issue 1: Gazebo RTF < 0.9

**Symptoms**: Gazebo simulation runs slowly (RTF < 0.9)

**Solutions**:
- Reduce physics update rate: Edit world file, set `<real_time_update_rate>500</real_time_update_rate>`
- Simplify collision meshes: Use primitive shapes (box, sphere, cylinder) instead of complex meshes
- Disable shadows: Edit world file, set `<shadows>false</shadows>`
- Check GPU drivers: Run `nvidia-smi` and ensure GPU is recognized

#### Issue 2: Unity URDF Import Fails

**Symptoms**: "URDF import failed" error in Unity console

**Solutions**:
- Verify URDF syntax: Run `check_urdf simple_humanoid.urdf`
- Check mesh file paths: Ensure mesh files are in correct relative paths
- Update URDF Importer package: Unity Package Manager → URDF Importer → Update
- Try importing a known-good URDF first (e.g., `01-myfirst.urdf`)

#### Issue 3: ROS-TCP-Connector Connection Timeout

**Symptoms**: Unity shows "Connection timeout" when connecting to ROS 2

**Solutions**:
- Start ROS TCP Endpoint: `ros2 run ros_tcp_endpoint default_server_endpoint`
- Check firewall: `sudo ufw allow 10000/tcp`
- Verify IP address: Use `127.0.0.1` for local connections
- Check ROS 2 environment: `echo $ROS_DOMAIN_ID` (should be consistent across terminals)

#### Issue 4: Python Package Import Errors

**Symptoms**: `ModuleNotFoundError: No module named 'stable_baselines3'`

**Solutions**:
- Activate virtual environment: `source ~/humanoid_robotics/rl_training/venv/bin/activate`
- Reinstall packages: `pip install stable-baselines3[extra] gymnasium`
- Check Python path: `which python3` (should point to venv)

### 9.2 Platform-Specific Issues

#### WSL2: Gazebo GUI Not Displaying

**Solution**:
```bash
# Check WSLg is enabled
echo $DISPLAY
# Should output :0 or similar

# If not, install WSLg
wsl --update
wsl --shutdown
# Restart WSL2
```

#### macOS: Gazebo Not Supported

**Solution**: Use Docker-based Gazebo or develop on a Linux VM (VirtualBox, Parallels).

```bash
# Docker-based Gazebo on macOS (no GUI support)
docker pull osrf/ros:humble-desktop
docker run -it --rm osrf/ros:humble-desktop gazebo --verbose
```

---

## 10. Next Steps

Once environment setup is complete:

1. ✅ **Validation**: Run `validate_environment.sh` and confirm all checks pass
2. ✅ **Read Specification**: Review `specs/002-module-2-digital-twin/spec.md`
3. ✅ **Read Data Model**: Review `specs/002-module-2-digital-twin/data-model.md`
4. ✅ **Read Contract**: Review `specs/002-module-2-digital-twin/contracts/chapter-content-contract.md`
5. ✅ **Read Plan**: Review `specs/002-module-2-digital-twin/plan.md`
6. ➡️ **Start Development**: Proceed to `/sp.tasks` to generate actionable task list
7. ➡️ **Execute Tasks**: Run `/sp.implement` to begin content generation with specialized subagents

---

## 11. References

### Official Documentation

- **ROS 2 Humble**: https://docs.ros.org/en/humble/
- **Gazebo Classic**: https://classic.gazebosim.org/
- **Unity Documentation**: https://docs.unity3d.com/2022.3/Documentation/Manual/
- **Unity Robotics Hub**: https://github.com/Unity-Technologies/Unity-Robotics-Hub
- **stable-baselines3**: https://stable-baselines3.readthedocs.io/
- **Gymnasium**: https://gymnasium.farama.org/

### Helpful Resources

- **ROS 2 Tutorials**: https://docs.ros.org/en/humble/Tutorials.html
- **Gazebo Tutorials**: https://classic.gazebosim.org/tutorials
- **Unity Learn**: https://learn.unity.com/
- **URDF Tutorial**: http://wiki.ros.org/urdf/Tutorials

---

**Quickstart Status**: ✅ **COMPLETE**
**Phase 1 Status**: ✅ **COMPLETE** (research.md, data-model.md, contracts/, quickstart.md)
**Next Phase**: Constitution re-check, then `/sp.tasks` for task generation
