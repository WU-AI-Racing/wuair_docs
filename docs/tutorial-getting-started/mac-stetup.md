---

sidebar_position: 10

---

# Mac Setup

## Pre-req Installation

### Homebrew Package Manager  

You will need a package manager to install the necessary dependencies for our software set up. Open terminal then past the follow command into the terminal:

```
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

To install packages use `brew install <package>`

### Docker Desktop
Download [Docker Desktop](https://www.docker.com/products/docker-desktop/) for the architecture you have on your computer.

## Installation

### 1. Create a working directory

```
mkdir ~/wuair
cd ~/wuair
```

### 2. Clone the system repository

```
git clone https://github.com/WU-AI-Racing/wuair_system.git
cd wuair_system
```

Then switch the mac os branch:

```
git checkout mac_os
```
---

## Docker Setup (VNC)

The system provides a **web-based VNC environment** with ROS 2 Humble and XFCE.

### 1. Build the Docker image

```
docker compose build vnc
```

### 2. Run the VNC container

```
docker compose up vnc
```

* The container runs a VNC server on port `5901` and a web-based interface (noVNC) on port `6080`.
* To stop the container:

```
docker compose down
```

### 3. Access the desktop

Open your browser and navigate to:

```
http://localhost:6080/vnc.html
```

You will have full access to the ROS2 XFCE desktop environment.

> **Note:** The VNC server runs without authentication by default. Use only on local or trusted networks.

---

## Running the Simulation

After connecting via VNC:

1. Open a terminal in the container.
2. For useful aliases run `source src/.rosbashrc.sh`. That will give you the following short-cut commands:

```
alias .r='source /opt/ros/humble/setup.bash'
alias .w='source /ros2_ws/install/setup.bash'
alias .ws='cd /ros2_ws'

# Build and clean
alias cb='cd /ros2_ws && colcon build'
alias cbc='cd /ros2_ws && rm -rf build install log && colcon build'
alias cl='cd /ros2_ws && rm -rf build install log'

# List ROS environment info
alias envr='env | grep ROS'

# Shortcuts
alias ll='ls -lah --color=auto'
alias gs='git status'
alias gp='git pull'
alias gd='git diff'

# Run tmux if available
alias tm='tmux attach || tmux'
```

3. Build the workspace:

```
.r
colcon build --symlink-install
.w
```

> **Note:** If you see a lot of warning message don't worry. Ignore them and move onto launching the simulation.


4. Launch the simulation:

```
ros2 launch eufs_launcher eufs_launcher.launch.py
```

5. Launch WUAIR file:

```
ros2 run wuair_launcher gui
```
