---
sidebar_position: 4
image: "/img/ros-logo.jpg"
---

# Using Docker and ROS Intro

:::tip[Reminder for help]

remember getting help is an email away. Don't spend 5 hours on a problem that might be a 5 minute fix. It is okay and expected for you to have problems. The link for getting in contact with us is [here](./help-me.md). You can always also contact your team lead.

:::

Build the Docker compose image with 
```bash
sudo usermod -aG docker $USER && docker-compose up -d --build
```
this might take a while.

:::danger[Important Warning]

If you get a warning like:
```bash
WARN[0000] The "SSH_AUTH_SOCK" variable is not set. Defaulting to a blank string.
invalid spec: :/ssh-agent: empty section between colons
```
don't panic or ignore it. This is one of those times warnings matter. go back to **Your first SSH key** and redo steps 3 and 4. If they both act as expected, retry the above command. If you still get a warning, reach out.

If you get a permissions denied error, try restarting your terminal and/or docker desktop. If the error persists, restart your computer. Docker Desktop can be finicky.
:::

Then run the container with

```bash
docker-compose exec humble bash
```

you should get a new shell once this executes.


ROS2 time :)

## Running simulation

After entering the docker image, run 

```bash
source src/.rosbashrc.sh
.r
colcon build --symlink-install
.w
```

`source src/.rosbashrc.sh` will get you the following aliases:


<details>
  <summary><strong>Click to show aliases</strong></summary>

  ```bash
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
</details> 



`colcon build --symlink-install` will build the entire workspace and take a while.

You should now be able to run the simulation with the following command:
```bash
ros2 launch eufs_launcher eufs_launcher.launch.py
```

:::danger[Important Warning part 2]
If you get a rendering error or other significant error output restart your computer. Docker Desktop can be finicky, again.
:::

To launch using the WUAIR stack using the GUI
```
ros2 run wuair_launcher gui
```

From here, press the checkbox that says something like Gazebo in the options, then press launch.
This should get you a new set of guis, one is a controller and the other two are visualization. 

import React from 'react';
import confetti from 'canvas-confetti';



export function ConfettiButton() {
  const shoot = () => {
    confetti({
      particleCount: 150,
      spread: 70,
      origin: { y: 0.6 }
    });
    const audio = new Audio('/wuair_docs/audio/confetti.mp3');
    audio.addEventListener('canplaythrough', () => audio.play());
  };
 const primaryBlue = "var(--ifm-color-primary)";
  return (
    <button
      onClick={shoot}
      style={{
        marginLeft: '8px',           
        padding: '4px 10px',         
        backgroundColor: primaryBlue,
        color: 'white',
        border: 'none',
        borderRadius: '4px',
        cursor: 'pointer',
        fontSize: '0.95rem',
        display: 'inline-block',
        verticalAlign: 'middle'
      }}
    >
      you deserve this!
    </button>
  );
}



<p>
  You did it! <ConfettiButton />
</p>
