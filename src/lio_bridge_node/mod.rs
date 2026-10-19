mod bridge;
mod geometry;
mod parameters;
mod state;
mod subscribers;
mod tf;

use std::error::Error;

use rclrs::*;
use ros_env::nav_msgs::msg::Odometry;
use ros_env::tf2_msgs::msg::TFMessage;

use parameters::Parameters;
use state::LioBridgeData;
use subscribers::Subscriptions;

/// LIO odometry provider — the robot's only odometry source.
///
/// Wheel odometry is gone (the Isaac Sim OmniGraph odom publishers are disabled and the real
/// chassis publishes none): the robot relies solely on Point-LIO lidar-inertial odometry. This
/// node turns the LIO chain (`map -> lio_odom -> lio_body`) into the planar chain the nav stack
/// consumes:
///
/// ```text
/// odom -> base_link   from pointlio/lio_odom, projected to 2D (lio_body is physically the
///                     lidar's IMU, <robot_id>/lidar_top_imu, so the static
///                     base_link -> lidar_top_imu extrinsic maps it to base)
/// odom                nav_msgs/Odometry republished for the twist consumers (controller_server,
///                     task_runner, robot_state); linear velocity from LIO (body frame),
///                     angular.z from the lidar IMU gyro — a deliberate choice, not a fallback
/// map -> odom         AMCL-style correction using the localizer's map -> lio_odom TF:
///                     map -> odom = P2D(map -> base) · inv(P2D(odom -> base))
/// ```
///
/// Everything is projected to 2D (x, y, yaw; z / roll / pitch zeroed) before broadcasting so the
/// planar nav stack never sees a tilted frame. All three outputs are produced by one timer at
/// `publish_rate`, independent of the ~10 Hz at which LIO actually updates — the timer republishes
/// the latest cached state.
///
/// # Threading
///
/// Everything runs on ONE rclrs [`Worker`]: the four subscriptions and the publish timer.
/// Callbacks on a worker never overlap, which gives the sample caches and the TF buffer exactly
/// the protection the C++ version gets from its default single-threaded executor, with no lock
/// to take. Nothing here blocks — the TF lookups are a few hash lookups against an in-memory map —
/// so a second worker would buy a lock and nothing else.
pub struct LioBridgeNode {
    // Fields drop in declaration order: the timer stops firing before the subscriptions and the
    // worker that their callbacks run on go away.
    _timer: WorkerTimer<LioBridgeData>,
    _subscriptions: Subscriptions,
    _worker: Worker<LioBridgeData>,
    // Parameters are undeclared when their handles drop, so they are kept for the node's life
    _parameters: Parameters,
    _node: Node,
}

impl LioBridgeNode {
    pub fn new(node: Node) -> Result<Self, Box<dyn Error>> {
        let (parameters, config) = Parameters::declare(&node)?;
        let publish_period = config.publish_period;

        // Reliable, depth 10 — the rclcpp `create_publisher("odom", 10)` default. Both the
        // SensorData (best-effort) subscribers of robot_state and the reliable nav odom
        // subscribers match it; a best-effort publisher would satisfy only the former.
        let odom_publisher: Publisher<Odometry> = node.create_publisher("odom".keep_last(10))?;
        // The TransformBroadcaster of this port. tf2_ros's own broadcaster QoS: reliable, depth
        // 100, volatile. Absolute, because TF is global.
        let tf_publisher: Publisher<TFMessage> = node.create_publisher("/tf".keep_last(100))?;

        let worker = node.create_worker(LioBridgeData::new(
            config,
            odom_publisher,
            tf_publisher,
            node.get_clock(),
            node.logger().clone(),
        ));

        let subscriptions = Subscriptions::create(&worker)?;
        // The default timer clock is steady time, i.e. the C++ `create_wall_timer`: the publish
        // cadence does not follow /clock even under use_sim_time. The stamps written into the
        // messages do — they come from the node clock.
        let timer = worker.create_timer_repeating(publish_period, |data: &mut LioBridgeData| {
            data.on_timer();
        })?;

        log_info!(
            node.logger(),
            "[LioBridge] LIO odometry provider initialized: odom -> base_link + odom topic from \
             pointlio/lio_odom, map -> odom from the localizer TF, every {publish_period:?}"
        );

        Ok(Self {
            _timer: timer,
            _subscriptions: subscriptions,
            _worker: worker,
            _parameters: parameters,
            _node: node,
        })
    }
}
