import os
import math
import yaml
import numpy as np
from scipy.spatial.transform import Rotation as R

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable, DeclareLaunchArgument
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def quat_to_rpy(qx, qy, qz, qw):
    # SciPy expects quaternion in (x, y, z, w) format
    q = np.array([qx, qy, qz, qw])

    # Create rotation object
    rot = R.from_quat(q)
    
    # Convert to roll, pitch, yaw (xyz intrinsic rotations)
    roll, pitch, yaw = rot.as_euler('xyz', degrees=False)

    return roll, pitch, yaw


def load_scene(npz_path):
    """
    Returns: (model_names: list[str], poses: list[list[float]])
    poses rows are [x,y,z,qx,qy,qz,qw]
    """
    data = np.load(npz_path, allow_pickle=True)

    poses = data["poses"]
    model_names = data["model_names"]

    # Ensure plain Python types
    model_names = [str(n) for n in model_names.tolist()]
    poses = poses.tolist()

    if len(model_names) != len(poses):
        raise RuntimeError(f"model_names length ({len(model_names)}) != poses length ({len(poses)})")

    return model_names, poses

def get_scene_number(yaml_path):
    if not os.path.exists(yaml_path):
        raise RuntimeError(f"Scene config YAML not found: {yaml_path}")

    # Open yaml file
    with open(yaml_path, "r") as f:
        cfg = yaml.safe_load(f) or {}

    return int(cfg["scene_number"])
    

pkg_share = get_package_share_directory('mnet_scenes_gazebo')

# Directory that contains 003_cracker_box/, 006_mustard_bottle/, etc.
ycb_root = os.path.join(pkg_share, 'models', 'ycb')

set_paths = [
    SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=ycb_root),
    SetEnvironmentVariable(name='GZ_SIM_MODEL_PATH', value=ycb_root),
]


def generate_launch_description():
    # Read scene_number from yaml file
    scene_yaml = os.path.join(pkg_share, "config", "scene.yaml")
    scene_number = get_scene_number(scene_yaml)

    # Gets list of object names and poses
    model_names, poses = load_scene(
        os.path.join(pkg_share, "scenes", f"{scene_number}.npz")
    )

    # Start Gazebo with an empty world
    gazebo = ExecuteProcess(
        cmd=[
            "ros2", "launch",
            "ros_gz_sim", "gz_sim.launch.py",
            "gz_args:=empty.sdf"
        ],
        output="screen",
    )

    # Spawn Objects
    spawn_nodes = []
    for i, (model_name, pose) in enumerate(zip(model_names, poses)):
        roll, pitch, yaw = quat_to_rpy(pose[3], pose[4], pose[5], pose[6])

        # folder is the model_name (ex. 006_mustard_bottle), sdf file matches the name without the numeric prefix (ex. mustard_bottle.sdf)
        sdf_base = model_name.split("_", 1)[1] if "_" in model_name else model_name
        sdf_path = os.path.join(ycb_root, model_name, f"{sdf_base}.sdf")

        spawn_nodes.append(
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-file", sdf_path,
                    "-name", f"object_{i}",
                    "-x", str(pose[0]),
                    "-y", str(pose[1]),
                    "-z", str(pose[2]),
                    "-R", str(roll),
                    "-P", str(pitch),
                    "-Y", str(yaw),
                ],
                output="screen",
            )
        )

    return LaunchDescription([
        *set_paths,
        gazebo,
        *spawn_nodes,
    ])
