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
rviz2
```

-----

## ⚡ Pro Tip: Create a Shortcut (Alias)

Typing that long docker command sucks. Make a shortcut so you only have to type `caprobo`.

**1. Open your bash configuration on the Pi (Host):**

```bash
nano ~/.bashrc
```

**2. Scroll to the bottom and paste this:**

```bash
alias caprobo='xhost +local:root && docker run -it --privileged --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v /dev:/dev -v ~/ros2_ws:/root/ros2_ws caprobo'
```

**3. Save and Apply:**

  * Press `Ctrl+O`, `Enter` to save.
  * Press `Ctrl+X` to exit.
  * Run `source ~/.bashrc` to activate it.

**4. Now just type:**

```bash
caprobo
```

-----

## 🆘 Troubleshooting

| Issue | Solution |
| :--- | :--- |
| **`rviz2: command not found`** | You forgot to `source /opt/ros/humble/setup.bash` |
| **`unable to open display ""`** | You are in a text-only SSH. Connect via **Remote Desktop**. |
| **`exec format error`** | You downloaded an Intel (AMD64) image. Use the `caprobo` image. |
| **Code changes not showing?** | Run `colcon build` and `source install/setup.bash`. |

-----

## 📦 Container Maintenance

If you install new packages (like `sudo apt install ros-humble-gazebo...`) inside the container, they will vanish when you exit unless you commit them.

**To save changes permanently:**

1.  Keep the container running.
2.  Open a **new** terminal on the Pi.
3.  Find the ID: `docker ps`
4.  Save it: `docker commit <CONTAINER_ID> caprobo`
