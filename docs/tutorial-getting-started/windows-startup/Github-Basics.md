---
sidebar_position: 3
---

# GitHub Setup

:::tip[WSL Reminder]

Make sure you are in WSL land for this. As a reminder, this is done via running `wsl` in the windows terminal.

:::


## SSH keys (much easier than it looks I promise)
Make a new directory for wuair code. 
```bash
mkdir ~/wuair
cd wuair
```

Clone the system repository, enter it and go to this branch of the repo. To do so (and to set up for later git automation) we will be using Github SSH keys. 

## Your first SSH key:

:::tip[If you already have an SSH key]

skip this if you have already generated and added an SSH key to your Github and are willing to use it for this club. However, if you do skip it, you'll have to change the filenames in some of the commands below to match your key name. 

:::


We won't be sharing the actual private key with the docker container, so security risks are minimal with this method.

1. To generate an SSH key, make sure you are following the **Linux** instructions and **using wsl**. For maximal convenience, name your key "git_key" (to match subsequent commands) and do not add a passphrase. **This is not security advice, do not do this in real life**, we should **never use this key in any public facing automation**. You have been warned. Follow [this tutorial](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent?platform=linux) to generate the key. 
2. To add the SSH key to your Github Account, use [this tutorial](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account) (again use the Linux tutorial, remember our key is named git_key, so replace any "id_somenumber" with "git_key").

## Creating the SSH Agent
3. If you did every step right, you should be able to run the following commands:
```bash
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/git_key
```
4. to check if they succeeded, run 
```bash
echo $SSH_AUTH_SOCK # should return like /tmp/ssh-ejfbvjksrb/agent.PID or something
```
If that worked, move on to the next step. you shouldn't have to specify any of your git information in the docker container anymore (yay) 

## Cloning the repo
We will now be cloning the repo using an SSH connection. The first time you do this, ssh will ask you to trust github, and github will give you some information as proof of their authenticity. You can safely write "yes" and move on.
```bash
git clone git@github.com:WU-AI-Racing/wuair_system.git
cd wuair_system && git checkout windows_wsl # move to this branch, so we aren't using the linux version of files
```

Damn, that was a lot of work. Don't worry, we're getting there. Move onto [Using Docker and ROS Intro](using-docker-plus-ros.md) after this page.
