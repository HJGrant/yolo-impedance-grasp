# 🤖 UR10_Final_Project

![ROS](https://img.shields.io/badge/ROS-catkin-22314E?logo=ros&logoColor=white)
![Docker](https://img.shields.io/badge/Docker-DevContainer-2496ED?logo=docker&logoColor=white)
![YOLOv8](https://img.shields.io/badge/YOLOv8-Object%20Detection-00FFFF)
![RealSense](https://img.shields.io/badge/Intel-RealSense-0071C5?logo=intel&logoColor=white)
![UR10](https://img.shields.io/badge/Robot-UR10-1E90FF)

## 🏷️ Name
**Multi-sensory Based Robot Dynamic Manipulation** final project – *Visual Servo Tracking and Picking of Unknown Objects.*

## 📝 Description
Real-time visual servoing and grasping of unknown objects on a torque-controlled UR10. Learned perception (YOLOv8) feeds a model-based adaptive controller, running closed-loop on real hardware. 🎯🦾

- 👁️ **Learned perception:** YOLOv8 instance segmentation on RealSense RGB-D frames turns the target's mask into a 3D point cloud, with median-based depth filtering to reject background points.
- 📐 **6-DoF pose estimation:** PCA on the segmented point cloud gives the object's position and principal axis, smoothed with an exponential filter for position and slerp for orientation.
- 📷 **Camera-to-robot calibration:** the camera pose is estimated automatically by averaging 100 detections of an ArUco marker fixed to the robot base.
- 🧠 **Adaptive model-based control:** a Slotine-Li adaptive controller in Cartesian space uses the UR10 dynamics regressor and adapts the robot's dynamic parameters online, with torque control at 500 Hz.
- 🛡️ **Robust tracking:** a damped pseudoinverse handles singularities, joint velocities are capped, and the robot holds its last pose if the target is lost.
- 🤏 **Autonomous grasping:** once the object has been still for 10 s, a state machine moves to a pre-grasp pose and closes the WSG-50 gripper.
- 🐳 **Reproducible setup:** everything runs inside a Docker dev container on ROS Noetic.

👥 Developed by Hamish Grant and Leo [surname] as the final project for *Multi-sensory Based Robot Dynamic Manipulation* at TUM.

## 🎬 Demo Video

[![Project Demo](media/msbrdm_optimized.gif)](https://youtu.be/2c5BIXtCKbk?si=SGGOZjihsi6IsASQ)

▶️ Click the GIF to watch on YouTube!

---

## 📋 Table of Contents
1. [Clone the repository](#1--clone-the-repository)
2. [Build and Launch the DevContainer](#2--build-and-launch-the-devcontainer)
3. [Connect to the UR10](#3--connect-to-the-ur10)
4. [Build the workspace](#4--build-the-workspace)
5. [Initialize the pipeline](#5--initialize-the-pipeline)
6. [Calibrate the camera pose](#6--calibrate-the-camera-pose)
7. [Run the tracking and picking pipeline](#7--run-the-tracking-and-picking-pipeline)

---

# 🧰 Requirements
- 🐳 Docker
- 📷 RGB-D Camera (Realsense)
- 🦾 UR10

---

## 🚀 Usage

### 1. 📥 Clone the repository
```bash
git clone https://gitlab.lrz.de/hjgrant/ur10_final_project.git
```

### 2. 🐳 Build and Launch the DevContainer
Make sure to adjust the path of the workspace in the `devcontainer.json` to the location where you cloned the repo to. Then press `ctrl+p` in VSCode and select `Dev Container: Rebuild and Reopen in Container`.

### 3. 🔌 Connect to the UR10
🌐 Make sure you have given your host machine a static IP in the same subnet as the UR10's IP address, we tested with a static IP of `192.168.1.3`. Make sure to set the correct ROSMASTER IP, an alias has been set to simplify this. Run the following inside the container,

```bash
rosmaster_ur10
```

📡 make sure you can ping the robot, try

```bash
ping 192.168.1.10
```

✋ and the gripper

```bash
ping 192.168.1.20
```

⚙️ make sure to change line 5 in `src/tum_ics_ur10_controller_tutorial/launch/configs/configUR10.ini` to `'real'`.

### 4. 🔨 Build the workspace
Navigate to `UR10_FINAL_PROJECT\` and run `catkin build`, then wait for the build process to finish. ⏳

### 5. 🧩 Initialize the pipeline
Inside the container, open one terminal and run `roscore` to start the ros master.

```bash
roscore
```

🖥️ Then open another terminal and run the script manager

```bash
roslaunch tum_ics_ur_robot_manager robot_script_manager_ur10.launch
```

✋ open another terminal and run the gripper driver

```bash
roslaunch wsg_50_driver wsg_50_tcp.launch
```

🏠 open another terminal and make sure you home the gripper

```bash
rosservice call /wsg_50_driver/homing
```

👀 open another terminal and run RViz for the visualization

```bash
roslaunch tum_ics_ur10_bringup bringUR10.launch
```

### 6. 📐 Calibrate the camera pose
📷 To calibrate the camera pose, point the Realsense camera at the ARUCO marker mounted on the robot's base link, then run the following to launch the camera, the object detection and the camera pose calibration

```bash
roslaunch my_visual_servoing visual_servo_realsense.launch
```

🔢 check the terminal output and wait until the calibration process has finished, i.e. 100 measurements of the ARUCO marker have been recorded.

> [!NOTE]
> If you do not see the calibration measurements counting up to 100, look at the output of the topic `/aruco_single` in RViz and move the camera until you see a frame projected onto the ARUCO marker.

✅ After the calibration procedure is completed there will be a message in the terminal that reads **"SUCCESS!"**.

> [!WARNING]
> Be sure not to move the camera otherwise you will have to repeat this calibration process.

### 7. 🎯 Run the tracking and picking pipeline
🎉 Now that you have successfully calibrated the camera pose, run the following launch file to start the tracking pipeline,

```bash
roslaunch tum_ics_ur10_controller_tutorial visual_servo_controller.launch
```

🍾 after the robot has moved into its starting configuration, you can introduce the target object (pre-configured to class `'bottle'`) into the workspace. The end-effector will track the object.

⏱️ Once you are ready for the robot to grasp the object, keep the position and orientation of the object still for **10s**. After 10s the robot will start a grasping routine. 🤏🦾
