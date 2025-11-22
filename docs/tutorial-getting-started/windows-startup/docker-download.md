---
sidebar_position: 2
---

# Docker Setup

We are using Docker to emulate the Linux distribution we'll be using on the real car. The below instructions will get you set up with all necessary docker software.

## Docker Download
You'll need to install [Docker Desktop](https://www.docker.com/products/docker-desktop/) and docker using a package manager in WSL. 

You might be able to get away with just the package manager, but you'll likely have to be starting your own docker deamons every time you want to use Docker. Basically, you better know what you're doing. 

### Docker Desktop
Download [Docker Desktop](https://www.docker.com/products/docker-desktop/) for the architecture you have on your computer.


:::tip[Finding your architecture]

press ctrl-shift-esc, click on performance on the left, then CPU and if Intel you're probably x86-64/AMD

:::

Start the Docker Desktop executable, and leave it on in the background. you can skip all the sign-up stuff, you __should not__ have to give it any personal information. 

### Package Manager Docker 

:::danger[Please read]

From now on everything, unless stated otherwise, should be run from WSL. If you get command not found errors, that could be why.

:::

Now that you have WSL, in a powershell terminal, run
```powershell
wsl
```
This will take you to WSL (likely Ubuntu, a distribution of Linux) land, where everything is linux. This will help a lot when it comes to getting the display forwarding set up, since WSL will take the display from our Docker image and handle the translation for us. Think of them like a necessary middleman. 

install docker using your package manager of choice. e.g.:
```bash 
sudo snap install docker
```
verify with `docker --version`


:::tip[Next Steps]

You should get a version and build specification from the previous command. If that and docker desktop gave you no issues, move onto [GitHub Basics](./Github-Basics.md)

:::
