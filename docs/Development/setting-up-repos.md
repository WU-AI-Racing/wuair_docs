---

sidebar_position: 1

---


`# Getting Started With ROS Development

## Cloning Repositories

Once you have completed the software installation guide and set up Git and Docker you can now begin getting started on writing you own code. Each department will have more detailed development guides but this will allow you to get started. 

Instead of using nested Git repos, called submodules, we use individual repositories for each department. To ensure each repository is up-to-date and using correct branch we will use **vcstool** to automate cloning and pulling.
`
:::tip[If these Git terms are unfamiliar please refer to the [Git tutorial](/docs/tutorial-getting-started/what-is-git.md)]
:::

To get started you will need to set up an SSH key for github following [this guide](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent). With the SSH key you can now import each repository in the WUAIR stack and ensure they are up to date. 

After entering the `ros2_ws` [via Docker](/docs/tutorial-getting-started/intro) enter the `/src` directory. 

:::important 
The /src directory is mounted in the docker container! This means that you can make changes to files within the `\src` on your local machine or within docker and it will remain the same between both.
:::

Next source ros with
```
. /opt/ros/humble/setup.bash
```

Then clone your desired using `vcstool`. 
Move to the source directory and import the `wuair.rosinstall` file:

```
vcs import < wuair.rosinstall
```

All the packages should now be imported into your source file. Your only need to run this command once. To update packages after this importing them run:

```
vcs pull src
```

:::important
This will pull only the main branch of each repository. 
:::



To test code in experimental branches the best approach we currently have found it this to be the easiest method.
Now in on you **local machine** 
1. Delete the cloned repository you want work on. EX: `rm -fr perception_pkg`
2. Clone the repository you want to work on EX: `git clone git@github.com:WU-AI-Racing/wuair_perception.git`
3. Go to you desired experiment branch with `git checkout BRANCH-NAME`

Then build the ros workspace with:

```
.r
colcon build --symlink-install
.w
```

or non-aliased commands:

```
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source /ros2_ws/install/setup.bash
```

Now you should be able to edit files on your local machine and see the update in Docker. After editing files you should not need to rebuild every time because of the simlink, but for major changes you may need to rebuild.

## Running code 

Make sure EUFS is running in another terminal. 
```
ros2 launch eufs_launcher eufs_launcher.launch.py
```

To run components individually (after building) use the following command with you're desired package:

```
ros2 run wuair_control ros
```
