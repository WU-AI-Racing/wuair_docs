---
sidebar_position: 6
---

# Linux Setup

ROS2 Software must be run in Ubuntu Jammy (22.04). To enable everyone to implement their code without installing an entirely new operating system we will make use of Docker. This will create a Docker container on your computer that's running Ubuntu. 

## Linux Installation:

Make a new directory for wuair code


```
mkdir ~/wuair
cd wuair
```

Clone the system repository

```
git clone https://github.com/WU-AI-Racing/wuair_system.git
```

Next enter the cloned directory
```
cd wuair_system
```

### Docker set up
Build the Docker compose image with 
```
docker-compose up -d --build
```

Then run the container with

```
docker-compose exec humble bash
```


## Running simulation

After entering the docker image run `source src/.rosbashrc.sh` for useful aliases such as:

```
alias .r='source /opt/ros/humble/setup.bash'
alias .w='source /ros2_ws/install/setup.bash'
alias .ws='cd /ros2_ws'

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
```

Now build the system

```
.r
colcon build --symlink-install
.w
```

or non-aliased commands:
```
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source /ros2_ws/install/setup.bash
```

You should now be able to run the simulation with the following command.

```
ros2 launch eufs_launcher eufs_launcher.launch.py
```

:::tip Display error
If you are getting a display error you can run the following command:
`xhost +local:root`
:::

### Developing in Docker

Our system is set up with the `src` directory mounted from your local machine and the docker container. This allows you to make change to any repositories and code within them on your local machine in your IDE of choice (EX: VsCode) on your local computer and see those change live in the Docker container. After major changes you may need to rebuild the ROS workspace with:

```
colcon build --symlink-install
```

## Cloning the stack
To clone the our stack you can use the automated tool, vcstool. To get started you will need to set up an SSH key for github following [this guide](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent). With the SSH key you can now import each repository in the WUAIR stack and ensure they are up to date. 

Move to the source directory and import the `wuair.rosinstall` file:

```
vcs import < wuair.rosinstall
```

All the packages should now be imported into your source file. Your only need to run this command once. To update packages after this importing them run:

```
vcs pull src
```

#### This will pull only the main branch of each repository. 
To test code in experimental branches the best approach we currently have found it this
1. Delete the cloned repository you want work on. EX: `rm -fr perception_pkg`
2. Clone the repository you want to work on EX: `git clone git@github.com:WU-AI-Racing/wuair_system.git`
3. make change to that repository in your branch.
