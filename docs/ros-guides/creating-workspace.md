---

sidebar_position: 1

---

# Creating a New ROS2 Node

This guide walks you through creating a new ROS 2 package, adding a simple publisher node, and configuring the build files.

## Step 1: Create Your New Package

First, navigate to the src directory of your ROS 2 workspace (the one you set up in ros_repo_setup_guide.md).

```
cd src
```

Now, use the ros2 pkg create command. This will generate a complete package directory for you. You need to choose between C++ and Python.

For a C++ Package (Recommended):
The --node-name option will automatically create a simple main.cpp file.
```
ros2 pkg create --build-type ament_cmake --node-name my_cpp_node my_cpp_pkg
```

For a Python Package:
The --node-name option will automatically create a simple .py file.

```
ros2 pkg create --build-type ament_python --node-name my_py_node my_py_pkg
```

This will create a new directory (e.g., `my_cpp_pkg/` or` my_py_pkg/`) inside src/ with all the necessary files.

## Step 2: Add Your Node's Code

The command in Step 1 creates a basic "Hello World" node. Let's replace it with a more useful "Publisher" node.

I've provided two example files:

`my_publisher_node.cpp (for C++)`

`my_publisher_node.py (for Python)`
For C++:
Copy the code from my_publisher_node.cpp and use it to replace the contents of src/my_cpp_pkg/src/my_cpp_node.cpp.

For Python:
Copy the code from my_publisher_node.py and use it to replace the contents of src/my_py_pkg/my_py_pkg/my_py_node.py.

## Step 3: Update Your Build & Package Files

Because our new node publishes a std_msgs/String, it has new dependencies. You must tell the build system about them.

For C++ (my_cpp_pkg):
You need to edit two files:

`package.xml`: Add dependencies for rclcpp (ROS Client Library for C++) and std_msgs.

See `package.xml.example` for what this looks like.

`CMakeLists.txt`: Tell CMake to find the std_msgs package and link it to your node.

See `CMakeLists.txt`.example for the required changes.

For Python (my_py_pkg):
You only need to edit one file:

`package.xml`: Add dependencies for rclpy (ROS Client Library for Python) and std_msgs.

See `package.xml`.example for what this looks like.

`setup.py`: This file defines your package's metadata. You need to make sure the entry_points section is correct.

See `setup.py.example` for a complete file.

## Step 4: Build and Run Your New Node

Navigate to your workspace root (e.g., /ros2_ws).

```
cd ..
```

Build your new package:

#### Build just your new C++ package
```
colcon build --packages-select my_cpp_pkg
```

#### Or build just your new Python package
```
colcon build --packages-select my_py_pkg
```

Source your workspace:

```
source install/setup.bash
```

Run your node!

#### Run the C++ node
```
ros2 run my_cpp_pkg my_cpp_node
```
#### Run the Python node
```
ros2 run my_py_pkg my_py_node
```

Check the output: In another terminal (after sourcing), you can "listen" to the topic your node is publishing:

```
ros2 topic echo /topic
```

You should see `"Hello, world! 1"`, `"Hello, world! 2"`, etc.