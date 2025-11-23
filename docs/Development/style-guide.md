---

sidebar_position: 2

---

# ROS Python Development & Style Guide

To maintain a clean, stable, and readable codebase, all contributors must adhere to the following development guidelines.

## 1. Version Control & Git Workflow

### Main Branch is Sacred
The `main` (or `master`) branch should **always** be deployable and compilable.
* **NEVER** push experimental code directly to `main`.
* **NEVER** commit broken code to `main`.

### Branching Strategy
Create a new branch for every feature, bug fix, or experiment. Use descriptive prefixes:
* `feature/add-lidar-node` (New functionality)
* `fix/camera-latency` (Bug repair)
* `experiment/new-pid-tuning` (Testing ideas that might not work)

### Commit Messages
Commit messages must be descriptive. A commit message should tell us *what* changed and *why*, not just that files were modified.

* **Bad:** `fix`, `update`, `working on stuff`, `typo`
* **Good:** `Fix lidar timestamp synchronization issue`, `Add ROS parameter for max_speed`, `Refactor image processing into separate function`

---

## 2. ROS Parameters vs. Hardcoded Values

Hardcoding values makes code rigid and requires recompilation/edits to change behavior.

### If a variable might need to be changed during testing (e.g., speed, PID constants, topic names, timers), it **must** be a ROS parameter.

### Example

**❌ BAD (Hardcoded):**

```python
class Mover(Node):
    def timer_callback(self):
        # If we want to change speed, we have to edit the code!
        msg.linear.x = 0.5 
        self.publisher.publish(msg)
```
✅ GOOD (ROS Parameters):

```python

class Mover(Node):
    def __init__(self):
        super().__init__('mover')
        # Declare the parameter with a default value
        self.declare_parameter('target_speed', 0.5)

    def timer_callback(self):
        # Get the current value (can be changed live via command line or launch file)
        speed = self.get_parameter('target_speed').value
        msg.linear.x = speed
        self.publisher.publish(msg)
```
## 3. No "Magic Numbers"

A "Magic Number" is a raw number used in code without context. It confuses readers and makes updating math difficult.

The Rule

Replace magic numbers with named constants (UPPER_CASE) or ROS parameters.

❌ BAD:
```python
# What does 1.57 mean? Why 0.3?
if distance < 0.3:
    turn_angle = 1.57
```
✅ GOOD:
```python

import math

STOP_DISTANCE_M = 0.3
TURN_ANGLE_RAD = math.pi / 2  # 90 degrees

if distance < STOP_DISTANCE_M:
    turn_angle = TURN_ANGLE_RAD
```
## 4. Modularity & Functions

Do not write "God Functions." A single function (especially a ROS callback) should not handle logic, calculation, logging, AND publishing.


### Split code into reusable, logical blocks. A function should do one thing well.

❌ BAD (The "Everything" Callback):
```python

def image_callback(self, msg):
    # 50 lines of code converting image
    # 20 lines of code detecting red pixels
    # 10 lines of code calculating center point
    # 5 lines of code publishing
    pass
```
✅ GOOD (Modular):
```python

def image_callback(self, msg):
    cv_image = self.convert_ros_to_cv2(msg)
    center_point = self.detect_red_object(cv_image)
    
    if center_point:
        self.publish_drive_command(center_point)

def convert_ros_to_cv2(self, msg):
    # Only handles conversion
    return cv_bridge.imgmsg_to_cv2(msg)

def detect_red_object(self, image):
    # Only handles detection logic
    # This logic can now be unit tested easily!
    return (x, y)
```
## 5. Python Coding Standards (PEP 8)

We follow standard Python PEP 8 style guides.

    Classes: CamelCase (e.g., `class CameraNode(Node)`:)

    Functions/Variables: snake_case (e.g., `def calculate_speed():, lidar_data = ...`)

    Constants: UPPER_CASE (e.g., `MAX_RPM = 5000`)

## 6. Logging

Don't use Python's print() function. Use the ROS logger. print() output often gets lost in ROS launch logs and doesn't support severity levels.

❌ BAD:
```python
print("Starting the robot...")
```
✅ GOOD:
```python
self.get_logger().info("Starting the robot...")
self.get_logger().warn("Battery low!")
self.get_logger().error("Sensor connection failed.")
```