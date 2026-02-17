# mnet_scenes_gazebo

## ROS 2 Jazzy package for importing ManipulationNet scenes into Gazebo

This package provides a simple way to load predefined ManipulationNet scenes into Gazebo using ROS 2 Jazzy.

---

## Installation
Clone the package into your ROS 2 workspace:

```
cd ~/ros2_ws/src
git clone https://github.com/JohnBrann/mnet_scenes_gazebo.git
```

## Build

```
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build && source install/setup.bash
```
## Usage

Select which scene to load by editing the configuration file at: **`config/scene.yaml`**.

Available scenes are listed in **scenes/metadata.json**

```yaml
scene_number: 5
```

Launch the selected scene in Gazebo

```
ros2 launch mnet_scenes_gazebo create_scene.launch.py
```