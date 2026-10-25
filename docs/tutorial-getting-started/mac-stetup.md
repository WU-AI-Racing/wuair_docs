---
sidebar_position: 10
---

# Mac Setup

Only the Docker setup is Mac-specific. Once it is done, building and running the stack is identical to Linux: follow [Running the Stack](/docs/Development/running-the-stack.md).

## 1. Install prerequisites

First do the shared [GitHub SSH Setup](./github-ssh-setup.md) (git, SSH key, test). Optionally install [Homebrew](https://brew.sh) for other tools.

Download [Docker Desktop for Mac](https://www.docker.com/products/docker-desktop/) (Apple Silicon build on M1-M4, Intel build otherwise), version 4.25 or newer.

## 2. Configure Docker Desktop

1. **Settings → General** (Apple Silicon only): confirm **Use Rosetta for x86_64/amd64 emulation on Apple Silicon** is checked. It is often on by default in recent versions, but if it is off Docker falls back to the much slower QEMU. It needs macOS 13+ and the Apple Virtualization framework as the VMM. Intel Macs can skip this.
2. **Settings → Resources → Network:** check **Enable host networking** (every container uses `network_mode: host`).
3. **Settings → Resources:** give Docker at least 6 CPUs and 8 GB RAM; the Gazebo image is heavy.
4. Apply & Restart.

Make sure macOS itself has Rosetta 2 (a no-op if it is already installed):

```bash
softwareupdate --install-rosetta --agree-to-license
```

Why Rosetta: Gazebo Classic 11 has no arm64 build, so `dev_gui` is pinned to `linux/amd64` and runs under CPU translation. Graphics are software-rendered inside the container and shown through your browser (noVNC), so nothing touches the macOS OpenGL stack. Expect a slow sim (roughly 3-15 fps).

## 3. Clone

Requires the SSH key from [GitHub SSH Setup](./github-ssh-setup.md).

```bash
mkdir ~/wuair && cd ~/wuair
git clone git@github.com:WU-AI-Racing/wuair_v2.git
cd wuair_v2
git submodule update --init --recursive
```

Use the default `main` branch; there is no separate Mac branch anymore.

## 4. Build (Mac-specific)

:::warning Build dev_gui with `--no-cache` on Mac
Gazebo's dependencies are fragile on macOS and a cached `dev_gui` layer can leave you with Gazebo errors. Build it fresh, either with the script:

```bash
./scripts/build.sh --profile dev_gui --no-cache
```

or directly:

```bash
docker compose --profile dev_gui build --no-cache dev_gui
```

The first build takes ~15 minutes. Then build the rest normally:

```bash
./scripts/build.sh --profile sensors --profile perception --profile state_est --profile controls
```
:::

:::danger Build `controls` with the script, not raw `docker compose build`
The `controls` container bind-mounts `controls/install/` over the image's workspace. That folder is not in git, so on a fresh clone it is empty and would hide the built packages, and the container would crash on start. `./scripts/build.sh` copies `install/` out of the freshly built image for you. If you ever built `controls` another way, fix it with:

```bash
./scripts/build.sh --profile controls
```

Also make sure `git submodule update --init --recursive` ran, or `controls/src/ros_can/FS-AI-API` is empty and the image build fails.
:::

`sensors`, `perception` and `state_est` are still being tested on Mac. If one fails to build, try `--no-cache` for that profile and tell the software lead. For perception on a Mac use `WUAIR_PERCEPTION_MODE=cpu` (the default).

## 5. Run

From here it is identical to every other OS: [Running the Stack](/docs/Development/running-the-stack.md).

## Troubleshooting

- **`no matching manifest for linux/arm64/v8`**: Rosetta emulation is off (step 2).
- **Gazebo errors / sim window never appears**: rebuild with the `--no-cache` command in step 4, then `./scripts/down.sh` and relaunch.
- **Stack never starts driving or topics flicker**: run `./scripts/down.sh` before relaunching after any `.env` change.
