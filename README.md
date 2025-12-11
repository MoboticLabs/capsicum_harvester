# CapRobo: ROS 2 Humble on Raspberry Pi 5 (Docker)

This repository runs inside a custom Docker container (`caprobo`) designed for the Raspberry Pi 5 (ARM64). It handles the OS mismatch (Ubuntu 24.04 Host vs. Ubuntu 22.04 Container) and provides full hardware access and GUI support (RViz).

## 🚀 Quick Start (Run this to work)

**1. Enable Graphics (Run on Host)**
*Required once per reboot/login.*

```bash
xhost +local:root
```

**2. Start the Container**
This command mounts your code (`~/ros2_ws`), enables the GPU, and gives access to USB/GPIO.

```bash
docker run -it --privileged --net=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev:/dev \
    -v ~/ros2_ws:/root/ros2_ws \
    caprobo
```

-----

## 🛠️ Development Workflow (Inside the Container)

Once you see the `root@...:/#` prompt, follow this cycle:

### 1\. Activate the Environment

You **must** run these two commands in every new terminal window:

```bash
# 1. Load ROS 2 Humble core
source /opt/ros/humble/setup.bash

# 2. Load your CapRobo workspace
source /root/ros2_ws/install/setup.bash
```

### 2\. Build Your Code

If you edit C++ or Python files in `src/`, rebuild the workspace:

```bash
cd /root/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 3\. Visualizing

To open RViz (ensure you are using Remote Desktop, not plain SSH):

```bash
ros2 launch robot_description rviz.launch.py
```

## 📦 Container Maintenance

If you install new packages (like `sudo apt install ros-humble-gazebo...`) inside the container, they will vanish when you exit unless you commit them.

**To save changes permanently:**

1.  Keep the container running.
2.  Open a **new** terminal on the Pi.
3.  Find the ID: `docker ps`
4.  Save it: `docker commit <CONTAINER_ID> caprobo`

## Launch Node:

To launch a node, for example, full_system.launch.py, you can use:

```bash
ros2 launch robot_control full_system.launch.py
```

## Manual IK Command

If you want to give the IK command manually, you can do it by using another terminal and giving coordinates like this:

```bash
ros2 topic pub /target_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'link1'},
  pose: {
    position: {x: 0.00, y: 0.12, z: 0.45},
    orientation: {x: 0, y: 0, z: 0, w: 1}
  }
}" --once
```

## Might Be Useful

**If you change the package.xml, then make sure to run the following (first run the last two lines if still not working, run the full code)**

```bash
apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -yapt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

**If you added a new launch file, don't forget to edit the CMakeList.txt file**

Current CMakeList.txt of controls: 

```bash
install(
  PROGRAMS scripts/ik.py scripts/detection_node.py scripts/serial_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
)
```
**Always make sure to make the script executable and then only build**

For Docker, the execution code needs to be like this:

```bash
chmod +x /root/ros2_ws/src/capsicum_harvester/robot_control/scripts/detection_node.py
```

**For checking which serial port to use for the launch file, use this:**

```bash
ls /dev/ttyACM* /dev/ttyUSB*
```

and to give read and write rights, use this

```bash
sudo chmod 666 /dev/ttyACM0
```
