

# **rrst-ros2-workspace**
## 1. Overview
This is the main repository for the NHK Project, RRST, at Ritsumeikan University, containing the ROS 2 packages and microcontroller-side programs currently in use.

|  | Description | Build Status |
|---|---|---|
| **main** | Stable branch (verified) | [![ROS 2 Jazzy Build](https://github.com/RRST-NHK-Project/rrst-ros2-ws/actions/workflows/main_jazzy_build_and_test.yml/badge.svg?branch=main)](https://github.com/RRST-NHK-Project/rrst-ros2-ws/actions/workflows/main_jazzy_build_and_test.yml) |
| **develop** | Latest development branch (in progress) | [![ROS 2 Jazzy Build](https://github.com/RRST-NHK-Project/rrst-ros2-ws/actions/workflows/main_jazzy_build_and_test.yml/badge.svg?branch=develop&event=push)](https://github.com/RRST-NHK-Project/rrst-ros2-ws/actions/workflows/main_jazzy_build_and_test.yml) |
| **Docker** | Docker build (develop) | [![Docker Build](https://github.com/RRST-NHK-Project/rrst-ros2-ws/actions/workflows/docker-publish.yml/badge.svg?branch=develop)](https://github.com/RRST-NHK-Project/rrst-ros2-ws/actions/workflows/docker-publish.yml) |

---

## 2. System Requirements

| Item | Description |
|:---|:---|
| OS | Ubuntu 24.04 LTS |
| ROS | ROS 2 Jazzy |
| RAM | 16 GB or more recommended |

> **Note**: If the build process freezes, it may be due to insufficient RAM.  
> Consider adding swap space or limiting the number of build threads.

---

## 3. Getting Started

### 3.1 Create a Workspace
Skip this step if the workspace has already been created.

    mkdir -p ~/ros2_ws/src

### 3.2 Clone the Repository

**main (stable)**

    cd ~/ros2_ws/src
    git clone https://github.com/RRST-NHK-Project/rrst-ros2-ws.git .
    git submodule update --init --recursive

**develop (latest, in progress)**

    cd ~/ros2_ws/src
    git clone https://github.com/RRST-NHK-Project/rrst-ros2-ws.git -b develop .
    git submodule update --init --recursive

### 3.3 Build

    cd ~/ros2_ws
    colcon build

To specify the number of build threads:

    cd ~/ros2_ws
    colcon build --parallel-workers 4

---

## 4. Directory Structure

| Path | Description |
|:---|:---|
| `/example` | Tutorials and training materials |
| `/ros2udp` | `ros2udp` package, used in ABU Robocon 2025 |
| `/ros2esp` | Test environment for ABU Robocon 2026 |
| `/nr26_r1_hw_ctrl` | Hardware control for R1, currently used in ABU Robocon 2026 |
| `/nr26_r2_hw_ctrl` | Hardware control for R2, currently used in ABU Robocon 2026 |
| `/serial_bridge` | Package for serial communication between ROS 2 and microcontrollers |
| `/microcontroller-ws` | Microcontroller programs (registered as a submodule) |

## 5. Credits
Developed by NHK Project, RRST, Ritsumeikan University, Japan (2024)
- Official Website: https://www.rrst.jp
- X (Twitter): https://x.com/RRST_BKC

![Logo](https://www.rrst.jp/img/logo.png)

---


