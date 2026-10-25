---
sidebar_position: 6
---

# Linux Setup

ROS 2 Humble runs in Ubuntu 22.04. To avoid installing a new OS we use Docker containers, so any Linux distro works.

## 1. GitHub SSH

Do the shared [GitHub SSH Setup](./github-ssh-setup.md) first.

## 2. Install Docker

Install Docker Engine and the compose v2 plugin following the [official guide](https://docs.docker.com/engine/install/), then let your user run it:

```bash
sudo usermod -aG docker $USER   # log out and back in
docker compose version          # should print v2.x
```

Only for `WUAIR_PERCEPTION_MODE=cuda`: install the NVIDIA Container Toolkit.

## 3. Clone and build

```bash
mkdir -p ~/wuair && cd ~/wuair
git clone git@github.com:WU-AI-Racing/wuair_v2.git
cd wuair_v2
git submodule update --init --recursive
./scripts/build.sh          # first time, and after any src/ change
```

Always build with `./scripts/build.sh`, not a bare `docker compose build`; the `controls` container needs the script to seed its `install/` folder.

## 4. Run

Identical on every OS: [Running the Stack](/docs/Development/running-the-stack.md).
