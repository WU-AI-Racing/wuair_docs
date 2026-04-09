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
---

## Docker Setup (VNC)

The system provides a **web-based VNC environment** with ROS 2 Humble and XFCE.

### 1. Build the Docker image
To build the image and start both the ROS environment and the VNC server in the background, run the following command from the root of the repository:

```bash
docker compose --profile mac up -d --build
```

### 2. Access the desktop

Open your browser and navigate to:

```bash
http://localhost:6080/vnc.html
```

You will have full access to the ROS2 XFCE desktop environment.

> **Note:** The VNC server runs without authentication by default. Use only on local or trusted networks.

---

### 3. Run the VNC container after building for the first time

```bash
docker compose --profile mac up -d
```

Open your browser and navigate to:

```bash
http://localhost:6080/vnc.html
```

* To stop the container:

```
docker compose --profile mac down
```

## Running the Simulation

After connecting via VNC:

1. Open a terminal in the container.
2. For useful aliases run
```
source src/.rosbashrc.sh`
```
or if using zsh
```
source src/.roshzshrc
```
That will give you the following short-cut commands:

:::tip

Don't paste this into your terminal. This is just a list of the commands and the alias contained in the contrainer.
:::

```
# Source ROS 2 and your workspace
alias .r='source /opt/ros/humble/setup.bash'
alias .w='source /ros2_ws/install/setup.bash'
alias .ws='cd /ros2_ws'
alias eufs='ros2 launch eufs_launcher eufs_launcher.launch.py'
alias wuair_gui='ros2 run wuair_launcher gui'

# Build and clean
alias cb='cd /ros2_ws && colcon build'
alias cbc='cd /ros2_ws && rm -rf build install log && colcon build'
alias cl='cd /ros2_ws && rm -rf build install log'1

# List ROS environment info
alias envr='env | grep ROS'


# Shortcuts
alias ll='ls -lah --color=auto'
alias gs='git status'
alias gp='git pull'
alias gd='git diff'

# Run tmux if available
alias tm='tmux attach || tmux'

# --- Auto-source workspace on container login ---
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Auto-source your own workspace if it's been built
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

export ROS_WORKSPACE=/ros2_ws
export ROS_DISTRO=humble
export EUFS_MASTER=/ros2_ws/src/eufs_master

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
