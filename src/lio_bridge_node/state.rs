//! The worker payload and the publish tick — the C++ node's member state and `timer_cb()`.
//!
//! Everything here runs on the node's single [`Worker`], so the LIO cache, the IMU cache, the TF
//! buffer and the cached extrinsic need no locking at all — the worker is rclrs's equivalent of an
//! rclcpp `MutuallyExclusive` callback group and guarantees its callbacks never overlap. The C++
//! gets the same guarantee from the default single-threaded executor.

use rclrs::*;
use ros_env::builtin_interfaces::msg::Time as TimeMsg;
use ros_env::geometry_msgs::msg::{
    Quaternion as QuaternionMsg, Transform as TransformMsg, TransformStamped, Vector3,
};
use ros_env::nav_msgs::msg::Odometry;
use ros_env::std_msgs::msg::Header;
use ros_env::tf2_msgs::msg::TFMessage;
use std::time::Duration;

use super::bridge;
use super::geometry::{Pose2D, Transform};
use super::parameters::Config;
use super::tf::{InvalidTransform, QUATERNION_NORM_TOLERANCE, TfBuffer};

/// The C++ throttles every "waiting for ..." line at 5 s; matched so the log reads the same.
const WAIT_LOG_THROTTLE: Duration = Duration::from_secs(5);
const SAMPLE_LOG_THROTTLE: Duration = Duration::from_secs(5);

/// Everything the node holds: the latest LIO and IMU samples, the TF tree, the cached lidar
/// extrinsic, one "has localized yet" flag, and the handles needed to publish.
pub struct LioBridgeData {
    config: Config,
    odom_publisher: Publisher<Odometry>,
    /// The `tf2_ros::TransformBroadcaster` of this port: a plain publisher on `/tf`.
    tf_publisher: Publisher<TFMessage>,
    clock: Clock,
    logger: Logger,

    tf: TfBuffer,

    /// `lio_odom -> lio_body`, from `pointlio/lio_odom`. `None` until the first message.
    lio_pose: Option<Transform>,
    /// The LIO odom frame, from the message header rather than a parameter: Point-LIO's
    /// `world_frame` is configured per setup (`pointlio_odom` on the Isaac config, `lio_odom`
    /// upstream), so taking it from the header means a rename upstream needs no change here.
    lio_odom_frame: String,
    /// Body-frame linear velocity from LIO's own twist.
    lio_linear: [f64; 2],
    /// Yaw rate from the lidar IMU gyro, not from LIO — see [`Self::accept_imu`].
    yaw_rate: f64,
    /// `base_link -> lidar_top_imu`, looked up once and cached: it is a static transform from the
    /// URDF, so there is nothing to re-read.
    base_lidar: Option<Transform>,
    /// Whether the first `map -> odom` has gone out. Only drives the one-shot "localization
    /// bridged" log line, which is the definitive "the robot is localized" signal.
    localized: bool,
}

impl LioBridgeData {
    pub fn new(
        config: Config,
        odom_publisher: Publisher<Odometry>,
        tf_publisher: Publisher<TFMessage>,
        clock: Clock,
        logger: Logger,
    ) -> Self {
        Self {
            config,
            odom_publisher,
            tf_publisher,
            clock,
            logger,
            tf: TfBuffer::default(),
            lio_pose: None,
            lio_odom_frame: String::new(),
            lio_linear: [0.0, 0.0],
            yaw_rate: 0.0,
            base_lidar: None,
            localized: false,
        }
    }

    // --- sample intake, one per subscription -------------------------------------------------

    /// The latest `pointlio/lio_odom`: the pose becomes `lio_odom -> lio_body`, the header's
    /// frame becomes the LIO odom frame, and the body-frame linear velocity is kept for the odom
    /// topic.
    ///
    /// **Difference from the C++ version:** a pose with a NaN or a non-unit quaternion is dropped
    /// here rather than cached. The C++ would compose it and broadcast a NaN transform, which tf2
    /// then rejects on every consumer with a far less helpful message. Dropping it leaves the
    /// previous pose in place, the same policy the TF buffer applies to a malformed transform.
    pub fn accept_lio(&mut self, frame_id: &str, pose: Transform, linear: [f64; 2]) {
        if !pose.is_finite() || (pose.rotation.norm() - 1.0).abs() > QUATERNION_NORM_TOLERANCE {
            log_warn!(
                self.logger.throttle(SAMPLE_LOG_THROTTLE),
                "[LioBridge] Ignoring a pointlio/lio_odom pose that is not a finite rigid \
                 transform; holding the previous one"
            );
            return;
        }
        self.lio_pose = Some(pose);
        if self.lio_odom_frame != frame_id {
            self.lio_odom_frame = frame_id.to_owned();
        }
        self.lio_linear = linear;
    }

    /// The latest `livox/imu` gyro z. The IMU is co-located with the lidar and the robot is
    /// treated as planar, so gyro z is the yaw rate.
    ///
    /// The raw gyro is used in preference to Point-LIO's own `twist.angular`, which
    /// `pointlio_node` does populate from its EKF output-model state `x().omg`. An earlier
    /// version of this comment claimed LIO left that field empty; it does not, and the estimate
    /// is not the bias-free one it looks like: `pointlio_params.yaml` sets `gyr_cov_output:
    /// 1000.0`, so the output model tracks the measurement almost instantly. Measured at
    /// standstill on robot01 (2026-09-21, 40 s):
    ///
    /// ```text
    /// Point-LIO twist.angular.z   mean +0.0620 deg/s   std 0.005672
    /// livox/imu gyro z (this)     mean +0.0907 deg/s   std 0.000964
    /// ```
    ///
    /// Same residual bias either way — and nothing downstream integrates it — but Point-LIO's
    /// is 5.9x noisier. Switching would only add noise.
    pub fn accept_imu(&mut self, gyro_z: f64) {
        self.yaw_rate = gyro_z;
    }

    pub fn accept_transform(&mut self, parent: &str, child: &str, transform: Transform) {
        if let Err(e) = self.tf.insert(parent, child, transform) {
            self.warn_invalid_transform(parent, child, e);
        }
    }

    fn warn_invalid_transform(&self, parent: &str, child: &str, reason: InvalidTransform) {
        log_warn!(
            self.logger.throttle(SAMPLE_LOG_THROTTLE),
            "[LioBridge] Ignoring TF '{parent}' -> '{child}': {reason}"
        );
    }

    // --- the tick ------------------------------------------------------------------------------

    /// One publish tick, in the C++ order: `odom -> base_link` and the odom topic first, which
    /// need only LIO and the extrinsic; then `map -> odom`, which additionally needs the
    /// localizer's correction and is skipped — not failed — while that is absent.
    ///
    /// The node deliberately does NOT wait for localization before publishing the odom chain,
    /// mirroring AMCL, where `odom -> base_link` exists before an initial pose is given. The
    /// robot can be driven and the odom-frame local costmap works before relocalize.
    pub fn on_timer(&mut self) {
        let Some(lio_pose) = self.lio_pose else {
            log_info!(
                self.logger.throttle(WAIT_LOG_THROTTLE),
                "[LioBridge] waiting for pointlio/lio_odom (LIO initializing?)"
            );
            return;
        };
        let Some(base_lidar) = self.base_lidar() else {
            return;
        };

        let chain = bridge::odom_to_base(lio_pose, base_lidar);

        let now = self.clock.now();
        // Future-dated by transform_tolerance, the same trick AMCL uses: consumers can
        // interpolate up to that far ahead without hitting an extrapolation error, which also
        // bridges the gap between LIO's ~10 Hz updates and this node's 20 Hz republishing.
        let tf_stamp = stamp_of(
            now.nsec
                .saturating_add(duration_nsec(self.config.transform_tolerance)),
        );

        // odom -> base_link: published before relocalization, like AMCL where the odom chain
        // exists before an initial pose.
        self.broadcast(
            &tf_stamp,
            &self.config.odom_frame,
            &self.config.base_frame,
            chain.odom_base,
        );

        // Odometry topic for the twist consumers (controller_server, task_runner, robot_state —
        // none of them read the pose; they take it from TF).
        self.publish_odom(stamp_of(now.nsec), chain.odom_base);

        // map -> odom: needs the localizer's map -> lio_odom, which only exists after
        // /<robot_id>/relocalize has been given a map (or the INI's [initial_pose] / an
        // initialpose message has done the same).
        let map_lio_odom = match self.tf.lookup(&self.config.map_frame, &self.lio_odom_frame) {
            Ok(t) => t,
            Err(e) => {
                log_info!(
                    self.logger.throttle(WAIT_LOG_THROTTLE),
                    "[LioBridge] waiting for TF (relocalized yet?): {e}"
                );
                return;
            }
        };

        let map_odom = bridge::map_to_odom(map_lio_odom, chain);
        self.broadcast(
            &tf_stamp,
            &self.config.map_frame,
            &self.config.odom_frame,
            map_odom,
        );

        if !self.localized {
            self.localized = true;
            log_info!(
                &self.logger,
                "[LioBridge] localization bridged: {} -> {} = ({:.3}, {:.3}, yaw {:.3})",
                self.config.map_frame,
                self.config.odom_frame,
                map_odom.x,
                map_odom.y,
                map_odom.yaw,
            );
        }
    }

    /// The cached static `base_link -> lidar_top_imu` mount extrinsic, looked up on first use.
    ///
    /// Until it resolves the node publishes nothing at all — not even the odom chain — because
    /// the whole tick depends on it. `lidar_frame` must therefore name a real frame: it comes
    /// from the URDF's `lidar_top_joint` and `lidar_top_imu_joint` (two static links, which the
    /// lookup walks), published by `syncai_bringup`.
    fn base_lidar(&mut self) -> Option<Transform> {
        if let Some(cached) = self.base_lidar {
            return Some(cached);
        }
        match self
            .tf
            .lookup(&self.config.base_frame, &self.config.lidar_frame)
        {
            Ok(t) => {
                self.base_lidar = Some(t);
                Some(t)
            }
            Err(e) => {
                log_info!(
                    self.logger.throttle(WAIT_LOG_THROTTLE),
                    "[LioBridge] waiting for {} -> {}: {e}",
                    self.config.base_frame,
                    self.config.lidar_frame,
                );
                None
            }
        }
    }

    /// One `TransformStamped` per `TFMessage`, exactly as `tf2_ros::TransformBroadcaster::
    /// sendTransform(const TransformStamped &)` does — so the two broadcasts of a tick arrive at
    /// listeners as two messages, as they do from the C++ node.
    fn broadcast(&self, stamp: &TimeMsg, frame_id: &str, child_frame_id: &str, pose: Pose2D) {
        let msg = TFMessage {
            transforms: vec![TransformStamped {
                header: Header {
                    stamp: stamp.clone(),
                    frame_id: frame_id.to_owned(),
                },
                child_frame_id: child_frame_id.to_owned(),
                transform: TransformMsg {
                    translation: Vector3 {
                        x: pose.x,
                        y: pose.y,
                        z: 0.0,
                    },
                    rotation: yaw_quaternion_msg(pose.yaw),
                },
            }],
        };
        if let Err(e) = self.tf_publisher.publish(msg) {
            log_error!(
                self.logger.throttle(SAMPLE_LOG_THROTTLE),
                "[LioBridge] publish /tf {frame_id} -> {child_frame_id} failed: {e}"
            );
        }
    }

    /// The odom topic: planar pose, LIO's body-frame linear velocity, and the IMU yaw rate.
    ///
    /// Covariances are never populated (both arrays stay all-zero), and the twist is copied from
    /// the LIO body frame into a message whose `child_frame_id` is `base_link` without rotating
    /// by the mount extrinsic — with the 0.25 rad mount pitch that is roughly a 3% underestimate
    /// of forward speed. Both are inherited from the C++ version and fine for the consumers
    /// listed above.
    fn publish_odom(&self, stamp: TimeMsg, pose: Pose2D) {
        let mut odom = Odometry {
            header: Header {
                stamp,
                frame_id: self.config.odom_frame.clone(),
            },
            child_frame_id: self.config.base_frame.clone(),
            ..Default::default()
        };
        odom.pose.pose.position.x = pose.x;
        odom.pose.pose.position.y = pose.y;
        odom.pose.pose.orientation = yaw_quaternion_msg(pose.yaw);
        odom.twist.twist.linear.x = self.lio_linear[0];
        odom.twist.twist.linear.y = self.lio_linear[1];
        odom.twist.twist.angular.z = self.yaw_rate;

        if let Err(e) = self.odom_publisher.publish(odom) {
            log_error!(
                self.logger.throttle(SAMPLE_LOG_THROTTLE),
                "[LioBridge] publish odom failed: {e}"
            );
        }
    }
}

/// `(0, 0, sin(yaw/2), cos(yaw/2))`, as the C++ `make_tf()` writes it: a planar rotation and
/// nothing else, so z / roll / pitch are zero by construction.
fn yaw_quaternion_msg(yaw: f64) -> QuaternionMsg {
    let (sin, cos) = (yaw / 2.0).sin_cos();
    QuaternionMsg {
        x: 0.0,
        y: 0.0,
        z: sin,
        w: cos,
    }
}

/// `builtin_interfaces/Time` from a nanosecond count, built by hand because the `ros-env` the
/// node's messages come from (0.2) is not the one rclrs's `Time::to_ros_msg` returns (0.3).
///
/// Euclidean division so a (pathological) negative count still yields a `nanosec` in range.
fn stamp_of(nsec: i64) -> TimeMsg {
    TimeMsg {
        sec: i32::try_from(nsec.div_euclid(1_000_000_000)).unwrap_or(i32::MAX),
        nanosec: u32::try_from(nsec.rem_euclid(1_000_000_000)).unwrap_or(0),
    }
}

/// A `Duration` as the `i64` nanoseconds `Time::nsec` counts in. Saturates instead of panicking
/// on a value a `Time` could not hold; the parameter clamp keeps it far from that anyway.
fn duration_nsec(d: Duration) -> i64 {
    i64::try_from(d.as_nanos()).unwrap_or(i64::MAX)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_stamp_splits_into_seconds_and_nanoseconds() {
        let stamp = stamp_of(1_700_000_000_123_456_789);
        assert_eq!(stamp.sec, 1_700_000_000);
        assert_eq!(stamp.nanosec, 123_456_789);
    }

    #[test]
    fn a_zero_clock_is_a_zero_stamp() {
        let stamp = stamp_of(0);
        assert_eq!((stamp.sec, stamp.nanosec), (0, 0));
    }

    /// A clock that has gone backwards past the epoch still produces a nanosec field in range,
    /// which is what a consumer validates.
    #[test]
    fn a_negative_count_keeps_nanosec_in_range() {
        let stamp = stamp_of(-1);
        assert_eq!(stamp.sec, -1);
        assert_eq!(stamp.nanosec, 999_999_999);
    }

    #[test]
    fn the_tolerance_is_added_in_nanoseconds() {
        assert_eq!(duration_nsec(Duration::from_millis(100)), 100_000_000);
        assert_eq!(duration_nsec(Duration::ZERO), 0);
    }

    #[test]
    fn the_broadcast_quaternion_is_a_pure_yaw() {
        let q = yaw_quaternion_msg(std::f64::consts::PI);
        assert_eq!(q.x, 0.0);
        assert_eq!(q.y, 0.0);
        assert!((q.z - 1.0).abs() < 1e-12);
        assert!(q.w.abs() < 1e-12);
    }
}
