---

sidebar_position: 1

---

# What is ROS?

## Overview

For a more detailed guide you can follow [this tutorial](https://roboticsbackend.com/what-is-ros/) 

Robot Operating System (ROS) is an open-source robotics suite. Although named an "Operating System," ROS is not an actual OS but a set of software frameworks for robot software development. ROS processes are represented in a graph architecture where processing takes place in **nodes** that may receive, post, and multiplex sensor data, control, state, planning, actuator, and other messages ([ROS Wikipedia](https://en.wikipedia.org/wiki/Robot_Operating_System)).

### Creating a ROS Node
In ROS 2, we usually create a class that inherits from `Node`. Here is a basic example of a Python node:

```python
import rclpy
from rclpy.node import Node

class MyRobotNode(Node):
    def __init__(self):
        # Initialize the node with the name 'my_robot_node'
        super().__init__('my_robot_node')
        self.get_logger().info('The node has started!')

def main(args=None):
    rclpy.init(args=args)
    node = MyRobotNode()
    rclpy.spin(node) # Keep the node running
    rclpy.shutdown()
```

## ROS Structure

One of the most important features of ROS is its modularity. Like a graph with nodes representing processes and lines representing communications, it can be organized in any arbitrary configuration.

ROS allows robots to communicate between their various algorithms (nodes) and also between nodes and hardware.

### Data Distribution Service

To facilitate communication, ROS uses the Data Distribution Service (DDS). DDS acts like a pipe, moving data throughout the ROS system.

![ROS Pipeline](/img/ros_pipeline.svg "ROS Diagram")

**There are 3 methods of communication within ROS:**

### 1\. Publishers and Subscribers

This is the communication system most of you will rely on. Like a radio station, a node can **publish** different topics that any other node can **subscribe** to.

  * **Topic:** The specific "channel" the node publishes to.
  * **Message (msg):** The type of information the topic sends.

For example, if there is a `camera.py` node receiving information from a camera, it will publish the `/front_camera` topic and send image messages. Then other nodes, like an object detection node and a mapping node, can subscribe to the `/front_camera` topic, and both will receive the image messages.

![ROS Pipeline](/img/pub_and_sub.svg "ROS Diagram")

#### Code Example: Publisher & Subscriber

```python
# PUBLISHER inside a Node class
# Publishes a String message to 'topic_name' every time it's called
self.publisher_ = self.create_publisher(String, 'topic_name', 10)
self.publisher_.publish("Hello ROS")

# SUBSCRIBER inside a Node class
# Listens to 'topic_name' and triggers 'listener_callback' when data arrives
self.subscription = self.create_subscription(
    String, 
    'topic_name', 
    self.listener_callback, 
    10)

def listener_callback(self, msg):
    self.get_logger().info(f'I heard: {msg.data}')
```

### 2\. Services

Services have two components: a **Service Client** and a **Service Server**. The service client sends a **request**. For example, a service client could send a message to swivel a camera, and then a camera mover server would send a response confirming it moved the desired amount.

![ROS Services](/img/ros_service.svg "ROS Diagram")

#### Code Example: Service Server

```python
# Create a service that listens on 'move_camera'
# When called, it runs the 'move_callback' function
self.srv = self.create_service(MoveCamera, 'move_camera', self.move_callback)

def move_callback(self, request, response):
    # Logic to move hardware goes here
    # Assume request has a field 'angle'
    self.get_logger().info(f'Moving camera to {request.angle} degrees')
    response.success = True
    return response
```

### 3\. Actions

The last way ROS communicates is through Actions. In an action, a client node sends a **goal** for the robot to perform a certain task. A controller node then interprets this goal and sends progress updates (or **feedback**) as the robot completes the goal.

Once the goal is complete, the controller will send a different message. This is called the **result** of the action.

![ROS Actions](/img/ros_actions.svg "ROS Diagram")

#### Code Example: Action Client

```python
# Sending a goal to a robot to drive 10 meters
goal_msg = Drive.Goal()
goal_msg.target_distance = 10.0

self._action_client.wait_for_server()
# Send goal and register a callback for feedback
self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
```

:::tip[Don't worry if this feels unintuitive]
As you start to work in the ROS ecosystem it will become much easier to understand.
:::

## ROS Parameters

Instead of needing to recompile to change values in our ROS nodes—like `maximum_speed` or `simulation_mode`—we can add **Parameters**. These allow us to switch these values live while the robot is running.

## ROS Bags

It can be useful to record data from the robot to allow us to get more realistic data than is possible in the simulation. This will be especially important to use because we don't have the physical car to test on yet. ROS Bags can record data from any nodes and allow us to "replay" that data later to test our algorithms.

## Building ROS2 Packages

It's essential that you understand the concept of workspaces, underlays, and overlays.

  * *Read about these concepts on the official [ROS2 wiki](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html#basics).*

The command `colcon build` is used to build packages. You should always be conscious of the workspace folder you're building in. Generally, this will be the `~/ros2_ws/` folder unless you are explicitly doing some experimental work elsewhere.

When you run `colcon build`, the ROS2 build system will traverse the file system recursively, find all packages, and compile all packages that have changed or have not been compiled. Packages are identified by folders that contain a `package.xml` file. Nested packaging is not allowed (i.e., packages cannot contain sub-packages).

```
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Building Specific Packages

The `--packages-up-to` argument selects a specific package to build and also informs colcon to build all of the packages that it depends on. It can be used as follows:

```
colcon build --symlink-install --packages-up-to PACKAGE_NAME
```

Other package selection commands are documented in the colcon docs.

## Useful ROS Commands

While is ROS in running in another terminal you can run any of the following commands in a separate docker terminal after sourcing with `source /opt/ros/humble/setup.bash`. 
  * `ros2 topic list` … lists all the topics that are being subscribed to and published.
  * `ros2 topic echo TOPIC` … displays the messages published to that topic.
  * `ros2 topic hz TOPIC` … shows the publishing rate of the topic.
  *  `ros2 topic info TOPIC` … shows information about a topic.
  *  `ros2 bag play SOME_BAG` … play a bag.
  *  `ros2 bag play SOME_BAG` --loop … if you want the bag to keep playing on loop.
  *  `ros2 run PACKAGE_NAME EXECUTABLE_NAME` … runs an executable from a package. For example, ros2 run slam slam_node launches the slam_node node in the slam package. The other node in the slam is the plotter node.
  *  `ros2 launch PACKAGE_NAME LAUNCH_FILE_NAME` … runs the specified launch file. For example, ros2 launch slam slam.launch.py.
  *  `ros2 run rqt_reconfigure rqt_reconfigure` … opens a Graphical User Interface (GUI) for dynamic reconfiguration of running ROS 2 nodes. This interface enables viewing and modifying node parameters in real time without restarting the nodes.
  *  `ros2 wtf AKA ros2 doctor` … shows warning from your system.
  *  `ros2 run rqt_graph rqt_graph` … generates a graph showing the topics and their subscription and publication connections.

