# EUFS Simulator

The EUFS sim (Gazebo Classic + `eufs_launcher`) runs inside the `wuair_dev_gui` container. Its source is tracked directly in `wuair_v2/dev_gui/src/eufs_sim` (it is not a submodule; we edit it freely).

## Start it

```bash
./scripts/launch_sim.sh
```

Open `http://localhost:6080/vnc.html`, choose a track / mission / car in the EUFS launcher, and click **Launch**. Then start the autonomy stack with `./scripts/real_launch.sh` (see [Running the Stack](/docs/Development/running-the-stack.md)).

Sim runs need `WUAIR_USE_SIM_TIME=1` in `.env` (the default). `dev_gui` is amd64 only; Apple Silicon uses Rosetta.

## EUFS messages

We use messages from the EUFS repository (`ConeArrayWithCovariance`, `CanState`, ...). They are installed as a Debian package from `deploy/vendor/ros-humble-eufs-msgs_{amd64,arm64}.deb`, regenerated with `scripts/build_eufs_msgs_deb.sh`.
