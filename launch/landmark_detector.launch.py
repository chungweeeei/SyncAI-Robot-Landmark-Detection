# Launch the landmark detector (syncai_common/srv/DetectLandmark on
# <robot_id>/detect_landmark).
#
# robot_id is read from the system config INI at launch time, the convention
# of every launch file in SyncAI-Robot-Workspace (whose read_robot_id() this
# copies: a launch file in one package does not import another's), and is
# used as the node namespace, so the feeder's image_raw/compressed and
# camera_info resolve under it. No frame is set here: a pose comes back in the
# camera's frame (the feeder's frame_id, already <robot_id>/...) unless the
# caller names a target_frame.
#
# use_sim_time comes only from the params file: a launch override placed
# after it would silently win over the YAML value.

import configparser
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch import logging as launch_logging
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Absolute, so the INI resolves whatever the cwd: ~/robot_ws is the workspace
# inside the robot container, where docker-compose bind-mounts the per-robot
# instance INI over config/system.ini.
DEFAULT_SYSTEM_INI = os.path.expanduser("~/robot_ws/config/system.ini")
FALLBACK_ROBOT_ID = "default_robot"

logger = launch_logging.get_logger("landmark_detector.launch")


def read_robot_id(config_path: str) -> str:
    config = configparser.ConfigParser()
    if not config.read(config_path):
        logger.warning(
            f"System config '{config_path}' not found; "
            f"falling back to robot_id '{FALLBACK_ROBOT_ID}'"
        )
        return FALLBACK_ROBOT_ID

    robot_id = config.get("system", "robot_id", fallback="").strip()
    if not robot_id:
        logger.warning(
            f"No [system] robot_id in '{config_path}'; "
            f"falling back to '{FALLBACK_ROBOT_ID}'"
        )
        return FALLBACK_ROBOT_ID

    return robot_id


def launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration("system_config").perform(context)
    robot_id = read_robot_id(config_path)

    node = Node(
        package="syncai_landmark_detection",
        executable="landmark_detector",
        namespace=robot_id,
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )
    return [node]


def generate_launch_description():
    pkg_share = get_package_share_directory("syncai_landmark_detection")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "system_config",
                default_value=DEFAULT_SYSTEM_INI,
                description="System config INI providing [system] robot_id",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(pkg_share, "params", "landmark_detector_params.yaml"),
                description="The detector plugins and the landmark registry",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
