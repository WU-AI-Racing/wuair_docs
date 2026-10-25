# WUAIR Coding Tutorial

## Intro

This tutorial is a brief overview of the different departments of WUAIR, the problems they seek to solve, and how we develop in a ROS 2 environment.

It will guide you through developing Python code in an IDE and integrating that workflow with our Docker setup. If you are confused about Docker, Git, or ROS that’s okay! Please see the guides we have on the documentation site as well as any other online resources.

Along with developing code you will also learn how we test code. After all, we don't have a physical car to drive in. Instead we will use the Edinburgh University Formula Student (EUFS) simulator to watch our code running. This will be a critical part of every team’s development process and crucial to understand.

## Getting Started

Matching our own software stack the tutorial code is intended to be completed moving from perception, state estimation, path planning, and control in that order. By the end of the tutorial you will have made a completely fully autonomous driving car! Though because of the simplicity if may not be the most preformant 🙃.

### Cloning The Repo

Follow the setup guide for your OS, which ends with `wuair_v2` cloned and built:

* [Windows](/docs/category/windows-setup)
* [Linux](/docs/tutorial-getting-started/software-installation)
* [Mac](/docs/tutorial-getting-started/mac-stetup)

The tutorial code lives in its own repo. Clone it next to `wuair_v2` (not inside it):

```
cd ~/wuair
git clone git@github.com:WU-AI-Racing/wuair_tutorial.git
```

:::tip
If you are missing branches run `git fetch --all`
:::

### IDE Setup

Open `wuair_v2` and `wuair_tutorial` in your IDE of choice (we recommend vscode; `code .` opens the current directory). Module source lives under `<module>/src/` and is bind-mounted into its container, so edits on your host show up in the container.

## Running ROS code & EUFS

To test your code we use the physics based EUFS simulator, which lets you watch the car drive and visualize the ROS topics you publish in [RViz](https://docs.ros.org/en/rolling/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html).

:::tip
If you are confused about any of the ROS terminology please see the [What is ROS?](/docs/tutorial-getting-started/what-is-ros.md) guide
:::

Start the sim and the stack as described in [Running the Stack](/docs/Development/running-the-stack.md):

```
./scripts/launch_sim.sh
./scripts/real_launch.sh
```

Open `http://localhost:6080/vnc.html`. In the EUFS launcher select the options below. If you are testing perception, uncheck the "Use Simulated Perception" box.

![EUFS Launcher](/img/EUFS_launcher.png)

This gives you the raw LiDAR data used by the clustering-based cone detection in this tutorial instead of EUFS's own perception, which simply publishes to `/cones`.

To visualize other topics click add in the bottom left of RViz:

![Add button](/img/add_button.png)

Then switch to the topics tab and select a topic, for example the Velodyne LiDAR points.

![Add topic](/img/add_topic.png)

:::tip
Use the EUFS simulated perception for testing state estimation, path planning and control if you are having trouble with the tutorial perception.
:::

## Coding Assignments

For each module open the directory corresponding to the team, i.e. `/perception`. In each module’s directory there will be another sub directory called `<module name>/<module name>_pkg/`. This will contain the two files you will need to work on for each module. The first is the logic and algorithms of the node contained in the python file named after the module. The other file is the ROS 2 node which may or may not have a task to complete depending on the module. Once you complete the missing code under each TODO will be ready to build and test your code.

Rebuild after changing source (see [Running the Stack](/docs/Development/running-the-stack.md)):

```
./scripts/build.sh --profile <module>
```

Config YAML changes only need `docker compose restart wuair_<module>`.

To run or inspect a single node, open a shell in its container:

```
docker exec -it wuair_controls bash
```

Container names are `wuair_sensors`, `wuair_perception`, `wuair_state_est`, `wuair_controls`. To see every node's output, attach to its tmux with `docker exec -it wuair_controls tmux attach -t controls`.

### Perception

Make sure "Use Simulated Perception" is off and you launch with the ADS-DV.

In this module, you will implement a LiDAR processing pipeline in `perception_pkg/perception.py`. The raw data comes in as a large array of points (x, y, z). Your goal is to filter this data and group points into objects (cones).

* Ground Removal: The LiDAR sees the floor as thousands of points. You need to implement a filter in `find_cones()` that removes any points below a certain Z-height (`ground_thresh`).

* Distance Filtering: The sensor also sees the car itself! You must apply a "Donut" filter to remove points that are too close (hitting the car body) or too far away (background noise).

* Clustering: Use [DBSCAN](https://scikit-learn.org/stable/modules/generated/sklearn.cluster.DBSCAN.html) to group the remaining floating points into clusters.

* Centroid Calculation: Calculate the mean X and Y of each cluster to find the center of the cone.

#### Visualizing Data:
To verify your code is working, you need to visualize the PointCloud in RViz.

click the Add button in the bottom left.

Select the By Topic tab.

Find and select the Velodyne Points topic.

### State Estimation

Launch using the launch script. Note that the files for this module are named `mapper.py` and `mapper_node.py` respectively.

In this module, you will implement a SLAM (Simultaneous Localization and Mapping) system. The perception module gives you cones relative to the car, but the car is moving!

* Update Pose (Dead Reckoning): In `mapper.py`, implement the math to update the car's global x, y, and yaw based on velocity and steering inputs. Remember that position updates depend on the trigonometry of your heading (cos for X, sin for Y).

* Transform Cones: You will convert the "local" cone coordinates (relative to the car) into "global" map coordinates using a 2D rotation matrix.

* Data Association: Implement `add_to_map_if_new()` to check if a cone you see is new or if it's one you've already mapped. This usually involves checking the distance to known cones.

### Path Planning

In this module you will take the cones you mapped with the state estimation package and plan a path through them.

* Midpoint Calculation: In `path_planner.py`, iterate through the blue cones and find the closest corresponding yellow cone. Calculate the midpoint between them to find the center of the track.

* Sorting: The midpoints might be discovered in a random order. Sort them by distance from the car so the path is continuous.

* Smoothing: The raw midpoints will be jagged. You will use scipy splines (splprep, splev) to interpolate a smooth curve through the points.

* Visualization: In `planning_node.py`, you will also implement the logic to publish these lines to RViz so you can debug the path.

### Control

Congrats on getting to the end! Now you just need to make the car move.

You will implement a Pure Pursuit controller in `control_pkg/pure_pursuit.py`.

* Lookahead Point: Find a point on the planned path that is a specific distance (`lookahead_dist`) away from the car.

* Steering Calculation: Calculate the curvature required to drive an arc from the car to that target point. Be careful with your trigonometry (arctan) and ensure you clamp values to the physical limits of the steering rack.

* Speed Control: Implement logic to slow the car down when the steering angle is high (cornering) and speed up on straights.

* Acceleration: In `pure_pursuit_node.py`, implement a P-Controller to calculate the acceleration required to reach your target speed.
