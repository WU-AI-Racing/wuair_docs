---
sidebar_position: 2
---

# Running the Stack

The stack lives in one monorepo, `wuair_v2`. It runs as five ROS 2 Humble containers on a shared `ROS_DOMAIN_ID` with host networking. Everything is driven by scripts in `scripts/`.

| Container          | Contents                                        |
|--------------------|-------------------------------------------------|
| `wuair_sensors`    | OAK-D camera, LiDAR relay                       |
| `wuair_perception` | YOLOv8 cone detection                           |
| `wuair_state_est`  | SLAM, lap counter, state machine                |
| `wuair_controls`   | path planner, pure pursuit, `ros_can`           |
| `wuair_dev_gui`    | EUFS sim + RViz, viewed in a browser via noVNC  |

Data flow: `sensors → perception (/cones) → state_est (/slam/*) → controls (/cmd) → car or sim`.

## Prerequisites

- Docker + `docker compose` v2 (Docker Desktop: enable *Settings → Resources → Network → Enable host networking*)
- NVIDIA Container Toolkit only if `WUAIR_PERCEPTION_MODE=cuda`
- Apple Silicon: `dev_gui` runs under Rosetta 2

## Clone and build

If you followed your OS setup guide you have already done this; it is repeated here for reference. It needs the [GitHub SSH key](/docs/tutorial-getting-started/github-ssh-setup.md) (on Windows, inside WSL).

```bash
git clone git@github.com:WU-AI-Racing/wuair_v2.git
cd wuair_v2
git submodule update --init --recursive
./scripts/build.sh
```

Re-run `./scripts/build.sh` whenever anything under `<module>/src/` changes.

:::important Always build with `./scripts/build.sh`
`controls` bind-mounts `controls/install/` (git-ignored, empty on a fresh clone) over the image's workspace. Only `build.sh` seeds it from the built image. A bare `docker compose build` leaves it empty and `wuair_controls` crashes on start. To rebuild just controls: `./scripts/build.sh --profile controls`.
:::

:::tip Mac users
Build `dev_gui` fresh to avoid Gazebo errors: `./scripts/build.sh --profile dev_gui --no-cache` or `docker compose --profile dev_gui build --no-cache dev_gui`. See [Mac Setup](/docs/tutorial-getting-started/mac-stetup.md).
:::

## Run in simulation

The default `.env` is sim-ready (`WUAIR_USE_SIM_TIME=1`, ground-truth cones, no OAK, no CAN).

```bash
./scripts/launch_sim.sh     # dev_gui only: EUFS launcher + RViz
./scripts/real_launch.sh    # the autonomy stack (all four modules)
```

Open `http://localhost:6080/vnc.html` (printed by the script). In the EUFS launcher pick a track, mission and car, then click **Launch**.

Useful flags for `real_launch.sh`: `--no-perception` (laptops without a GPU), `--with-gui` (also start `dev_gui`).

Stop everything with `./scripts/down.sh`.

## Run on the car

1. Edit `.env` per `.env.example`: `WUAIR_USE_SIM_TIME=0`, `WUAIR_USE_GT_CONES=0`, `WUAIR_ENABLE_OAK=1`, LiDAR topic, `WUAIR_YOLO_WEIGHTS=/ros_ws/weights/best.pt`.
2. `./scripts/can_up.sh` (the CAN interface is `can_pcan`, never hardcode `can0`).
3. `./scripts/down.sh` then `./scripts/real_launch.sh`.
4. Optional RViz in a browser: `./scripts/viz.sh`.

:::warning
Container env is captured at *create*, not restart. After editing `.env`, run `./scripts/down.sh` before relaunching, or a stale `WUAIR_USE_SIM_TIME` will make `ros_can` fight the sim.
:::

On the vehicle the stack auto-starts at boot via systemd (`wuair-system`, `ros-bag`, `pcan-can-up`); see `deploy/systemd/` in the repo.

## Editing code

- **Config** (`<module>/config/*.yaml`): edit, then `docker compose restart wuair_<svc>`. No rebuild.
- **Source** (`<module>/src/`): edit, then `./scripts/build.sh`.
- **Shared messages**: edit only `shared/wuair_msgs/`. Copies under `<module>/src/wuair_msgs/` are overwritten on build.

## Getting a shell in a container

Containers are named `wuair_<module>` (`wuair_sensors`, `wuair_perception`, `wuair_state_est`, `wuair_controls`, `wuair_dev_gui`). ROS is already sourced in the shell:

```bash
docker exec -it wuair_controls bash
ros2 topic list
ros2 topic echo /cmd
```

One-off command without an interactive shell: `docker exec wuair_controls bash -lc "ros2 topic list"`. List running containers with `docker ps`; exit with `exit` (the container keeps running).

## Debugging

Attach to a container's tmux to see every node's output (`ctrl+b w` = window picker):

```bash
docker exec -it wuair_state_est tmux attach -t state_est
docker exec -it wuair_controls  tmux attach -t controls
```

Live tuning (writes to `<module>/logs/tuned_*.yaml`):

```bash
docker exec -it wuair_state_est python3 /ros_ws/config/tune.py
docker exec -it wuair_controls  python3 /ros_ws/config/tune.py
```

Recording and replaying bags: `./scripts/bag_record.sh`, `./scripts/bag_play.sh`.

The full README and `.claude/reference/env-vars.md` in `wuair_v2` list every `WUAIR_*` variable.
