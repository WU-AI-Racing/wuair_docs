---
sidebar_position: 1
---

# Getting Started with Windows -- Installing WSL


This guide is for Windows people running WSL (Windows Subsystem for Linux). If you do not know what any of that means/just want to get your workspace set up with minimal hassle on Windows, you're at the right place. 

Read this slowly and take your time. This is not supposed to be quick, but it is supposed to be doable for all skill levels. That being said,  ___Look at the bottom of this README if you need help___ . This might be wizardry to some of you, so ask for help from others. 

If you do know what you are doing, help the guy next to you. Good luck :)

## Requirements
You will need Windows 11 or some later iteration of Windows 10. Aside from that, enough storage for the docker container itself and all other dependencies we will have to install.

## WSL Install

get [WSL](https://learn.microsoft.com/en-us/windows/wsl/install) (really just run the first command you see). To ensure you have GUI rendering capabilities, run this in a windows powershell terminal:
```powershell
wsl --version
```
you should see something like this somewhere in the output: 

WSLg version: 1.0.xx

if that line is missing, that would not be ideal. Maybe message me (bottom of the README), since WSL would not have display forwarding capabilities, so you would not be able to use a lot of the later commands unmodified. 

This is usually due to a version mismatch (wrong version of Windows, maybe you downloaded the wrong version of WSL by accident), so try reinstalling first.

## Don't want WSL? 

:::tip[Optional Section]

This is if you know what you are doing and have a good reason for being here. If not, proceed onwards to [Docker Download](./docker-download.md).

:::

If you want to avoid WSL, I would recommend giving up. 

If you're adamant, investigate [X410](https://x410.dev/download/) (this one is much better) or [VcXsrv](https://sourceforge.net/projects/vcxsrv/). I can help up to a point but please understand you may be investigating OpenGL version mismatch errors and/or graphics card restrictions/antiquated function calls. Any LLM is capable of walking you through the setup for __X410/VcXsrv__, I will leave that to them. 

You will need to make modifications to your [Dockerfile](https://github.com/WU-AI-Racing/wuair_system/blob/windows_wsl/Dockerfile) and [docker-compose.yml](https://github.com/WU-AI-Racing/wuair_system/blob/windows_wsl/docker-compose.yml). I have completed this process to some success but again this is so much harder.

To run GUI applications from the container, your system (without WSL) needs to handle X11 forwarding. Using X410/VcXsrv:

    1.  Launch it (e.g., from the Start Menu).
    2.  for VcXsrv: In the "Display settings" wizard, make sure to check **"Disable access control"**.



