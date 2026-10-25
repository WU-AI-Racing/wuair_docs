---
sidebar_position: 1
---

# Getting Started With ROS Development

:::info
We no longer use per-department repositories or **vcstool**. All code lives in one monorepo, [`wuair_v2`](https://github.com/WU-AI-Racing/wuair_v2).
:::

Set up in this order: [GitHub SSH Setup](/docs/tutorial-getting-started/github-ssh-setup.md), then your OS guide ([Windows](/docs/tutorial-getting-started/windows-startup/installing-wsl.md), [Mac](/docs/tutorial-getting-started/mac-stetup.md), [Linux](/docs/tutorial-getting-started/software-installation.md)), which ends with `wuair_v2` cloned and built.

Each module has its own folder (`sensors/`, `perception/`, `state_est/`, `controls/`, `dev_gui/`) with its ROS packages under `src/` and its tunable YAML under `config/`. Work on a branch in this one repo; there is no more deleting and re-cloning individual repos.

Then follow [Running the Stack](./running-the-stack.md) to build and launch.
