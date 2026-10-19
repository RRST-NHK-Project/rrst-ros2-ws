

# **rrst-ros2-workspace**

[日本語](README.md) | **English**

## 1. Overview
This is the main repository for the NHK Project, containing the ROS 2 packages currently in use. It is intended to be forked for each competition.

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

### 3.2 Register an SSH Key
The repository and its submodules are managed over **SSH**. Skip this step if you have already registered a key.

    ssh-keygen -t ed25519 -C "your_email@example.com"
    cat ~/.ssh/id_ed25519.pub

Add the displayed public key on GitHub via [Settings → SSH and GPG keys](https://github.com/settings/keys) → **New SSH key**, then verify the connection:

    ssh -T git@github.com

You are done when you see `Hi <username>! You've successfully authenticated...`. See the [Wiki](https://github.com/RRST-NHK-Project/rrst-ros2-ws/wiki/SSH鍵の登録) for details.

### 3.3 Clone the Repository

**main (stable)**

    cd ~/ros2_ws/src
    git clone git@github.com:RRST-NHK-Project/rrst-ros2-ws.git .
    git submodule update --init --recursive

**develop (latest, in progress)**

    cd ~/ros2_ws/src
    git clone git@github.com:RRST-NHK-Project/rrst-ros2-ws.git -b develop .
    git submodule update --init --recursive

### 3.4 Build

    cd ~/ros2_ws
    colcon build

To specify the number of build threads:

    cd ~/ros2_ws
    colcon build --parallel-workers 4

---

## 4. Credits
Developed by NHK Project, RRST, Ritsumeikan University, Japan (2024)
- Official Website: https://www.rrst.jp
- X (Twitter): https://x.com/RRST_BKC

![Logo](https://www.rrst.jp/img/logo.png)

---

