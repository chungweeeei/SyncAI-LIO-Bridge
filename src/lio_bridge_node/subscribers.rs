//! The four subscriptions, all on the node's single worker.
//!
//! `pointlio/lio_odom` and `livox/imu` are relative so they inherit the `<robot_id>` namespace,
//! like every other topic in the stack. `/tf` and `/tf_static` are absolute because TF is global
//! in ROS 2 — which is why the launch file prefixes the FRAME names (`<robot_id>/base_link`)
//! instead: frames are how this stack separates robots that share one `/tf`.
//!
//! The QoS of each one mirrors its publisher. Getting one wrong is silent — a reliable subscriber
//! simply never matches a best-effort publisher and receives nothing at all — so check with
//! `ros2 topic info /<robot_id>/<topic> --verbose` rather than by reading the code.

use rclrs::*;
use ros_env::geometry_msgs::msg::{Pose, Quaternion as QuaternionMsg, Vector3};
use ros_env::nav_msgs::msg::Odometry;
use ros_env::sensor_msgs::msg::Imu;
use ros_env::tf2_msgs::msg::TFMessage;

use super::geometry::{Quaternion, Transform};
use super::state::LioBridgeData;

/// tf2's own listener QoS: depth 100 and reliable, with `/tf_static` additionally transient-local
/// so the static transforms published before this node started are still delivered. Without the
/// transient-local the `base_link -> lidar_top` extrinsic — published exactly once by
/// `robot_state_publisher`, usually long before this node starts — would never arrive.
pub const TF_DEPTH: u32 = 100;

pub struct Subscriptions {
    _lio: WorkerSubscription<Odometry, LioBridgeData>,
    _imu: WorkerSubscription<Imu, LioBridgeData>,
    _tf: WorkerSubscription<TFMessage, LioBridgeData>,
    _tf_static: WorkerSubscription<TFMessage, LioBridgeData>,
}

impl Subscriptions {
    pub fn create(worker: &Worker<LioBridgeData>) -> Result<Self, RclrsError> {
        Ok(Self {
            // From Point-LIO. Sensor-data QoS (best-effort) on both sensor subscriptions: it is
            // compatible with either a reliable (lio_node) or a best-effort publisher.
            _lio: worker.create_subscription(
                "pointlio/lio_odom".sensor_data_qos(),
                |data: &mut LioBridgeData, msg: Odometry| {
                    data.accept_lio(
                        strip_leading_slash(&msg.header.frame_id),
                        pose_to_transform(&msg.pose.pose),
                        [msg.twist.twist.linear.x, msg.twist.twist.linear.y],
                    );
                },
            )?,

            // From the Livox MID360 driver.
            _imu: worker.create_subscription(
                "livox/imu".sensor_data_qos(),
                |data: &mut LioBridgeData, msg: Imu| {
                    data.accept_imu(msg.angular_velocity.z);
                },
            )?,

            // The TransformListener of this port: tf2_ros's own QoS on both topics.
            _tf: worker.create_subscription(
                "/tf".keep_last(TF_DEPTH),
                |data: &mut LioBridgeData, msg: TFMessage| {
                    accept_tf_message(data, msg);
                },
            )?,
            _tf_static: worker.create_subscription(
                "/tf_static".keep_last(TF_DEPTH).transient_local(),
                |data: &mut LioBridgeData, msg: TFMessage| {
                    accept_tf_message(data, msg);
                },
            )?,
        })
    }
}

/// Both TF topics land here: the buffer expires nothing, so it has no reason to tell static
/// transforms from dynamic ones (see `tf.rs`).
fn accept_tf_message(data: &mut LioBridgeData, msg: TFMessage) {
    for stamped in &msg.transforms {
        data.accept_transform(
            strip_leading_slash(&stamped.header.frame_id),
            strip_leading_slash(&stamped.child_frame_id),
            to_transform(&stamped.transform.translation, &stamped.transform.rotation),
        );
    }
}

/// `geometry_msgs` translation + rotation -> rigid transform (the C++ `to_transform()`).
fn to_transform(translation: &Vector3, rotation: &QuaternionMsg) -> Transform {
    Transform {
        translation: [translation.x, translation.y, translation.z],
        rotation: Quaternion {
            x: rotation.x,
            y: rotation.y,
            z: rotation.z,
            w: rotation.w,
        },
    }
}

/// `geometry_msgs/Pose` -> rigid transform (the C++ `pose_to_transform()`).
fn pose_to_transform(pose: &Pose) -> Transform {
    Transform {
        translation: [pose.position.x, pose.position.y, pose.position.z],
        rotation: Quaternion {
            x: pose.orientation.x,
            y: pose.orientation.y,
            z: pose.orientation.z,
            w: pose.orientation.w,
        },
    }
}

/// ROS 2 frame ids carry no leading slash, but a bridged or hand-written ROS 1 publisher still
/// emits one. tf2 strips it for the same reason: otherwise `/map` and `map` are two frames and the
/// lookup fails with nothing in the log to explain why. Applied to the LIO header frame too, so
/// that frame and the localizer's child frame compare equal however each was spelled.
fn strip_leading_slash(frame_id: &str) -> &str {
    frame_id.strip_prefix('/').unwrap_or(frame_id)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_ros1_style_frame_id_loses_its_slash() {
        assert_eq!(strip_leading_slash("/map"), "map");
        assert_eq!(strip_leading_slash("map"), "map");
        assert_eq!(
            strip_leading_slash("/robot01/pointlio_odom"),
            "robot01/pointlio_odom"
        );
        assert_eq!(strip_leading_slash(""), "");
    }

    #[test]
    fn a_pose_message_becomes_the_same_transform_as_its_fields() {
        let pose = Pose {
            position: ros_env::geometry_msgs::msg::Point {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
            orientation: QuaternionMsg {
                x: 0.0,
                y: 0.0,
                z: 0.5,
                w: 0.8660254,
            },
        };
        let t = pose_to_transform(&pose);
        assert_eq!(t.translation, [1.0, 2.0, 3.0]);
        assert_eq!(t.rotation.z, 0.5);
        assert_eq!(t.rotation.w, 0.8660254);

        let same = to_transform(
            &Vector3 {
                x: 1.0,
                y: 2.0,
                z: 3.0,
            },
            &pose.orientation,
        );
        assert_eq!(same, t);
    }
}
