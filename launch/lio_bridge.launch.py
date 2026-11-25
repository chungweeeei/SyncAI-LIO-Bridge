# Launch the syncai_lio_bridge node — the robot's only odometry source.
# From Point-LIO (pointlio/lio_odom + the IMU its body frame is) it publishes
# the planar odom -> <robot_id>/base_link TF, the <robot_id>/odom Odometry
# topic, and the map -> <robot_id>/odom correction (replacing AMCL). There is
# no wheel odometry: the Isaac Sim OmniGraph odom publishers are disabled.
#
# robot_id is read from the system config INI at launch time and is used as
# the node namespace and as the TF prefix for the odom chain frames. The map
# frame stays "map".
#
# The INI's [system] robot_model (g23 when absent) picks
# params/models/<robot_model>.yaml (2026-11-25, the layout syncai_robot_state
# and syncai_driver_manager have): which URDF link Point-LIO's body frame is
# (lidar_frame) and which topic that IMU is on (imu_topic). Both were written
# into this launch and the node for the G23; on an M20, whose URDF has no
# lidar_top_imu and whose IMU is the vendor's /IMU, the node waited for its
# extrinsic for good and published nothing. A model with no file, or one that
# leaves out a key, fails the launch: the code defaults are the G23's, and
# another robot on them has no odometry and says so only in a throttled INFO.

import configparser
import os

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch import logging as launch_logging
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Absolute path so the INI resolves no matter what cwd the launch is started
# from — the old relative path only worked because every entrypoint happened
# to run from the workspace root. ~/robot_ws is the workspace inside the robot
# container, where docker-compose bind-mounts the per-robot instance INI over
# config/system.ini.
DEFAULT_SYSTEM_INI = os.path.expanduser("~/robot_ws/config/system.ini")
FALLBACK_ROBOT_ID = "default_robot"

# Every INI before [system] robot_model existed is a G23's (the workspace's
# launches read the key the same way).
DEFAULT_ROBOT_MODEL = "g23"

# What a model's file must set; a missing key would silently be the G23's.
MODEL_KEYS = ("lidar_frame", "imu_topic")

logger = launch_logging.get_logger("lio_bridge.launch")


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


def read_robot_model(config_path: str) -> str:
    """[system] robot_model of the INI, `g23` when the key (or the file) is missing."""
    config = configparser.ConfigParser()
    config.read(config_path)
    model = config.get("system", "robot_model", fallback="").strip().lower()
    if not model:
        logger.warning(
            f"No [system] robot_model in '{config_path}'; reading it as '{DEFAULT_ROBOT_MODEL}'"
        )
        return DEFAULT_ROBOT_MODEL
    return model


def model_params(model: str) -> tuple[str, dict]:
    """params/models/<model>.yaml in this package's share, and its parameters.

    Raises when the file is missing, unreadable, or leaves one of MODEL_KEYS
    out or empty. A new model's file needs a colcon build of this package
    before a launch finds it.
    """
    path = os.path.join(
        get_package_share_directory("syncai_lio_bridge"), "params", "models", f"{model}.yaml"
    )
    if not os.path.isfile(path):
        raise RuntimeError(
            f"robot_model '{model}' has no params/models/{model}.yaml in syncai_lio_bridge; "
            "add one (see params/models/g23.yaml) and rebuild, or fix [system] robot_model"
        )
    try:
        with open(path, encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        params = (data.get("/**") or {}).get("ros__parameters") or {}
    except (OSError, yaml.YAMLError, AttributeError) as e:
        raise RuntimeError(f"robot_model '{model}': '{path}' is unreadable ({e})") from e
    # Empty counts as missing, unlike syncai_robot_state's optional topics:
    # this node cannot run without either.
    missing = [k for k in MODEL_KEYS if not params.get(k)]
    if missing:
        raise RuntimeError(
            f"robot_model '{model}': '{path}' leaves {missing} unset under /** "
            "(the code defaults are the G23's)"
        )
    logger.info(
        f"robot_model '{model}': {path} (lidar_frame '{params['lidar_frame']}', "
        f"imu_topic '{params['imu_topic']}')"
    )
    return path, params


def launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration("system_config").perform(context)
    robot_id = read_robot_id(config_path)
    model_file, model = model_params(read_robot_model(config_path))

    use_sim_time = LaunchConfiguration("use_sim_time").perform(context) == "true"

    lio_bridge_node = Node(
        package="syncai_lio_bridge",
        executable="lio_bridge_node",
        name="lio_bridge_node",
        namespace=robot_id,
        output="screen",
        # The model's file first, then the inline dict, so a later entry wins
        # key by key: the file gives imu_topic, and its lidar_frame is the
        # bare link name the dict prefixes.
        parameters=[
            model_file,
            {
                "use_sim_time": use_sim_time,
                "map_frame": "map",
                "base_frame": f"{robot_id}/base_link",
                "odom_frame": f"{robot_id}/odom",
                # The IMU that Point-LIO's body frame is, as the model's URDF
                # names it (the G23's lidar_top_imu, under lidar_top; the
                # M20's imu), prefixed here like the other frames. Written
                # into this launch as lidar_top_imu until 2026-11-25.
                "lidar_frame": f"{robot_id}/{model['lidar_frame']}",
                "publish_rate": 20.0,
                "transform_tolerance": 0.1,
            }
        ],
    )

    return [lio_bridge_node]


def generate_launch_description():
    declare_system_config = DeclareLaunchArgument(
        "system_config",
        default_value=DEFAULT_SYSTEM_INI,
        description="Path to the system INI file providing [system] robot_id and robot_model",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock (Isaac Sim publishes /clock)",
    )

    return LaunchDescription(
        [
            declare_system_config,
            declare_use_sim_time,
            OpaqueFunction(function=launch_setup),
        ]
    )
