
# ROS 2 Jazzy Setup Guide

This guide provides step-by-step instructions for setting up ROS 2 Jazzy, Visual Studio Code, Gazebo Harmonic, and other essential tools on Ubuntu.

---

## Step 1: Install ROS 2 Jazzy

Follow the official installation guide:
[https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

---

## Step 2: Install Visual Studio Code (VS Code)

for raspberry pi

```bash
sudo apt update
sudo apt install wget gpg
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -o root -g root -m 644 packages.microsoft.gpg /usr/share/keyrings/
sudo sh -c 'echo "deb [arch=amd64 signed-by=/usr/share/keyrings/packages.microsoft.gpg] \
https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
sudo apt update
sudo apt install code
```

for ubuntu

You can install Visual Studio Code via the Ubuntu Software (App Center)

You can launch VS Code using:

```bash
code
```

---

## Step 3: Install Gazebo Harmonic

Follow the official installation instructions:
[https://gazebosim.org/docs/harmonic/install_ubuntu/](https://gazebosim.org/docs/harmonic/install_ubuntu/)

---

## Step 4: Install Colcon

```bash
sudo apt install python3-colcon-common-extensions -y
```

---

## Step 5: Create a ROS 2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build --symlink-install
```

---

## Step 6: Setup ROS Environment

Option A: Automatically source ROS 2 in every terminal:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

Option B: Edit .bashrc manually:

```bash
sudo apt install gedit
gedit ~/.bashrc
```

Add at the end of the file:

```bash
source /opt/ros/jazzy/setup.bash
```

Apply the changes:

```bash
source ~/.bashrc
```

---

## Step 7: Install ROS 2 Dependencies

```bash
sudo apt install git ros-dev-tools -y
sudo apt install libserial-dev -y
sudo apt install \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-controller-manager \
  ros-jazzy-diff-drive-controller \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro \
  ros-jazzy-twist-mux \
  ros-jazzy-twist-stamper \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-rviz2 \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-navigation2 \
  ros-jazzy-turtlebot3* 
sudo rosdep init
rosdep update
```

---

## Step 8: Hokuyo LiDAR Setup

Clone the ROS 2 driver for Hokuyo LiDAR:

```bash
cd ~/ros2_ws/src
git clone --recursive https://github.com/1R-ws/urg_node2
rosdep update
rosdep install -i --from-paths urg_node2

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Run the LiDAR node:

```bash
ros2 launch urg_node2 urg_node2.launch.py
```

---

## Step 9: Clone Protobot Packages

```bash
cd ~/ros2_ws/src
git clone https://github.com/1R-ws/diffdrive_arduino
git clone https://github.com/1R-ws/twist_stamper
git clone https://github.com/1R-ws/serial
git clone https://github.com/1R-ws/protobot
```

Build the workspace and source:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

**Note:** Check the [Protobot repository](https://github.com/1R-ws/protobot) to see instructions on how to run the robot.

---

## Step 10: Setting Permissions for /dev/ttyACM0

If you get a permission error:

```bash
# Check current permissions
ls -l /dev/ttyACM0

# Add your user to the dialout group
sudo usermod -aG dialout $USER

# Reboot your system
```

If it still does not work, apply temporary permissions:

```bash
sudo chmod 666 /dev/ttyACM0
```


