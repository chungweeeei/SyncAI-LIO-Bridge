//! The TF lookup the C++ node gets from `tf2_ros::Buffer` + `TransformListener`.
//!
//! rclrs has no `tf2_ros` binding, so the node subscribes `/tf` and `/tf_static` itself and feeds
//! the transforms into [`TfBuffer`]. [`TfBuffer::lookup`] then answers the two questions this node
//! asks: where is `lidar_frame` in `base_frame` (the mount extrinsic, looked up once and cached),
//! and where is the LIO odom frame in `map_frame` (the localizer's correction, looked up every
//! tick) — both at the latest available time.
//!
//! Nothing here touches ROS types — the subscription converts at the boundary — so the whole
//! module is unit-testable without a ROS environment or the robot. It is the same buffer
//! syncai_robot_state carries, minus the pose helper that node needs and this one does not.
//!
//! # What this deliberately does not do
//!
//! * **No time history, no interpolation.** Only the newest transform per child frame is kept.
//!   The C++ looks up at `tf2::TimePointZero`, which tf2 resolves to the newest sample anyway, so
//!   a history would never be read.
//! * **No separate static cache.** tf2 stores `/tf_static` in a cache that never expires; this
//!   buffer expires nothing at all, so the distinction has nothing to do. Both subscriptions feed
//!   one map and the last publisher of a given child frame wins, as in tf2.
//! * **No cache expiry.** tf2 drops entries older than its 10 s cache window but never drops the
//!   newest one for a frame, so a frozen publisher leaves a stale transform in place there too:
//!   if the localizer dies after relocalizing, both versions keep bridging its last correction.
//!
//! # The node hears its own broadcasts
//!
//! The `/tf` subscription receives the `odom -> base_link` and `map -> odom` this node publishes,
//! exactly as a `tf2_ros::TransformListener` would. They land in the buffer and lengthen the
//! chains the lookups walk (after the first broadcast `base_link`'s parent is `odom`), but they
//! never change an answer: both frames of a lookup still meet at a common root.

use std::collections::HashMap;
use std::fmt;

use super::geometry::Transform;

/// How far up the tree a walk may go before the tree is declared malformed. TF is a tree, so a
/// real chain is a handful of links; only a publisher that made a frame its own ancestor can
/// exceed this, and without the bound that walk would not terminate.
const MAX_CHAIN_DEPTH: usize = 100;

/// How far a quaternion's norm may stray from 1 before the transform is rejected. Same tolerance
/// tf2's `assertQuaternionValid` uses.
pub const QUATERNION_NORM_TOLERANCE: f64 = 1e-6;

/// Why a transform was not stored. The caller logs it; the buffer keeps its previous value for
/// that frame, the same way tf2's `setTransform` returns false and changes nothing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InvalidTransform {
    /// `header.frame_id == child_frame_id`. A frame cannot be its own parent.
    SelfParent,
    /// Either frame name is empty.
    EmptyFrameId,
    /// A NaN or an infinity somewhere in the translation or the rotation.
    NonFinite,
    /// The rotation is not a unit quaternion — including the all-zero one a default-constructed
    /// `geometry_msgs/Quaternion` carries, which is the common way this fires.
    NotUnitQuaternion,
}

impl fmt::Display for InvalidTransform {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let text = match self {
            Self::SelfParent => "frame_id equals child_frame_id",
            Self::EmptyFrameId => "frame_id or child_frame_id is empty",
            Self::NonFinite => "translation or rotation contains NaN/inf",
            Self::NotUnitQuaternion => "rotation is not a unit quaternion",
        };
        f.write_str(text)
    }
}

/// Why a lookup failed. All three mean the same thing to the node — keep waiting — but they say
/// very different things to whoever reads the log.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum LookupError {
    /// Nothing has ever been published about this frame. On this stack it is the normal state
    /// before `syncai_localizer` has been relocalized (`map` has never been mentioned) and before
    /// `syncai_bringup` is up (`lidar_top` has never been mentioned).
    UnknownFrame(String),
    /// Both frames are known but sit in trees that do not meet — usually a half-built TF graph.
    NotConnected { target: String, source: String },
    /// A frame is its own ancestor. Only a misbehaving publisher produces this.
    CycleDetected(String),
}

impl fmt::Display for LookupError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnknownFrame(frame) => write!(f, "frame '{frame}' is unknown to TF"),
            Self::NotConnected { target, source } => {
                write!(f, "no TF path from '{source}' to '{target}'")
            }
            Self::CycleDetected(frame) => {
                write!(f, "TF tree has a cycle at or above '{frame}'")
            }
        }
    }
}

struct Link {
    parent: String,
    /// `parent <- child`, where the child is the map key.
    transform: Transform,
}

/// The newest transform per child frame, and the tree walk over them.
#[derive(Default)]
pub struct TfBuffer {
    links: HashMap<String, Link>,
}

impl TfBuffer {
    /// Store one transform, replacing whatever was last heard for `child`.
    ///
    /// A frame has exactly one parent in TF, so the child is the key. Two publishers claiming the
    /// same child fight over this slot — which is a broken TF graph, and tf2 behaves the same way.
    pub fn insert(
        &mut self,
        parent: &str,
        child: &str,
        transform: Transform,
    ) -> Result<(), InvalidTransform> {
        if parent.is_empty() || child.is_empty() {
            return Err(InvalidTransform::EmptyFrameId);
        }
        if parent == child {
            return Err(InvalidTransform::SelfParent);
        }
        if !transform.is_finite() {
            return Err(InvalidTransform::NonFinite);
        }
        if (transform.rotation.norm() - 1.0).abs() > QUATERNION_NORM_TOLERANCE {
            return Err(InvalidTransform::NotUnitQuaternion);
        }

        self.links.insert(
            child.to_owned(),
            Link {
                parent: parent.to_owned(),
                transform,
            },
        );
        Ok(())
    }

    /// Whether anything has ever been published about this frame, as a parent or as a child.
    fn knows(&self, frame: &str) -> bool {
        self.links.contains_key(frame) || self.links.values().any(|link| link.parent == frame)
    }

    /// Walk up to the root, returning it together with `root <- frame`.
    /// The returned name is either `frame` itself (when nothing is published about its parent) or
    /// a borrow out of the buffer, so both have to share one lifetime.
    fn chain_to_root<'a>(&'a self, frame: &'a str) -> Result<(&'a str, Transform), LookupError> {
        let mut current = frame;
        // Starts as `frame <- frame`, and grows into `current <- frame` as the walk goes up
        let mut to_frame = Transform::default();

        for _ in 0..MAX_CHAIN_DEPTH {
            let Some(link) = self.links.get(current) else {
                return Ok((current, to_frame));
            };
            // `parent <- current` composed with `current <- frame` gives `parent <- frame`
            to_frame = link.transform.then(to_frame);
            current = &link.parent;
        }

        Err(LookupError::CycleDetected(frame.to_owned()))
    }

    /// `target <- source`: the source frame expressed in the target's coordinates.
    ///
    /// This is what `tf2_ros::Buffer::lookupTransform(target, source, TimePointZero)` returns.
    pub fn lookup(&self, target: &str, source: &str) -> Result<Transform, LookupError> {
        if !self.knows(target) {
            return Err(LookupError::UnknownFrame(target.to_owned()));
        }
        if !self.knows(source) {
            return Err(LookupError::UnknownFrame(source.to_owned()));
        }
        if target == source {
            return Ok(Transform::default());
        }

        let (target_root, to_target) = self.chain_to_root(target)?;
        let (source_root, to_source) = self.chain_to_root(source)?;
        if target_root != source_root {
            return Err(LookupError::NotConnected {
                target: target.to_owned(),
                source: source.to_owned(),
            });
        }

        // (root <- target)^-1 composed with (root <- source) is (target <- source). Going via the
        // root rather than the lowest common ancestor gives the same answer: the extra links
        // above the common ancestor appear once in each chain and cancel.
        Ok(to_target.inverse().then(to_source))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::lio_bridge_node::geometry::Quaternion;
    use std::f64::consts::FRAC_PI_2;

    fn transform(translation: [f64; 3], yaw: f64) -> Transform {
        Transform {
            translation,
            rotation: Quaternion::from_yaw(yaw),
        }
    }

    fn assert_close(actual: f64, expected: f64) {
        assert!(
            (actual - expected).abs() < 1e-9,
            "expected {expected}, got {actual}"
        );
    }

    fn assert_planar_close(actual: Transform, x: f64, y: f64, z: f64, yaw: f64) {
        assert_close(actual.translation[0], x);
        assert_close(actual.translation[1], y);
        assert_close(actual.translation[2], z);
        assert_close(actual.rotation.yaw(), yaw);
    }

    #[test]
    fn a_single_link_is_returned_as_published() {
        let mut buffer = TfBuffer::default();
        buffer
            .insert("base_link", "lidar_top", transform([0.2, 0.0, 0.5], 0.75))
            .unwrap();

        assert_planar_close(
            buffer.lookup("base_link", "lidar_top").unwrap(),
            0.2,
            0.0,
            0.5,
            0.75,
        );
    }

    /// The real chain on this stack after relocalize: the localizer publishes
    /// map -> pointlio_odom, Point-LIO publishes pointlio_odom -> pointlio_body, and this node
    /// asks for map -> pointlio_odom while a sibling asks for map -> base_link.
    #[test]
    fn a_two_link_chain_composes_rotation_and_translation() {
        let mut buffer = TfBuffer::default();
        // map -> odom: a quarter turn, no offset
        buffer
            .insert("map", "odom", transform([0.0, 0.0, 0.0], FRAC_PI_2))
            .unwrap();
        // odom -> base_link: 2 m along odom's x, which the quarter turn swings onto map's y
        buffer
            .insert("odom", "base_link", transform([2.0, 0.0, 0.0], 0.0))
            .unwrap();

        assert_planar_close(
            buffer.lookup("map", "base_link").unwrap(),
            0.0,
            2.0,
            0.0,
            FRAC_PI_2,
        );
    }

    #[test]
    fn a_lookup_against_the_walk_direction_is_the_inverse() {
        let mut buffer = TfBuffer::default();
        buffer
            .insert("map", "base_link", transform([1.0, 2.0, 0.0], FRAC_PI_2))
            .unwrap();

        let there = buffer.lookup("map", "base_link").unwrap();
        let back = buffer.lookup("base_link", "map").unwrap();
        assert_planar_close(there.then(back), 0.0, 0.0, 0.0, 0.0);
    }

    /// The two frames meet below the root, so the links above the meeting point must cancel.
    #[test]
    fn a_lookup_between_two_branches_goes_through_the_common_ancestor() {
        let mut buffer = TfBuffer::default();
        buffer
            .insert("map", "odom", transform([5.0, 5.0, 0.0], 1.0))
            .unwrap();
        buffer
            .insert("odom", "base_link", transform([1.0, 0.0, 0.0], 0.0))
            .unwrap();
        buffer
            .insert("odom", "lidar", transform([0.0, 3.0, 0.0], 0.0))
            .unwrap();

        assert_planar_close(
            buffer.lookup("base_link", "lidar").unwrap(),
            -1.0,
            3.0,
            0.0,
            0.0,
        );
    }

    /// What happens once this node's own broadcasts come back through `/tf`: base_link acquires
    /// a parent, and the extrinsic lookup must still answer the same.
    #[test]
    fn the_nodes_own_broadcasts_do_not_change_the_extrinsic() {
        let mut buffer = TfBuffer::default();
        buffer
            .insert("base_link", "lidar_top", transform([0.2, 0.0, 0.5], 0.1))
            .unwrap();
        let before = buffer.lookup("base_link", "lidar_top").unwrap();

        buffer
            .insert("odom", "base_link", transform([7.0, -3.0, 0.0], 2.0))
            .unwrap();
        buffer
            .insert("map", "odom", transform([1.0, 1.0, 0.0], -0.5))
            .unwrap();
        let after = buffer.lookup("base_link", "lidar_top").unwrap();

        for i in 0..3 {
            assert_close(after.translation[i], before.translation[i]);
        }
        assert_close(after.rotation.yaw(), before.rotation.yaw());
    }

    #[test]
    fn a_frame_is_at_the_origin_of_itself() {
        let mut buffer = TfBuffer::default();
        buffer
            .insert("map", "base_link", transform([1.0, 2.0, 3.0], 1.0))
            .unwrap();

        assert_eq!(buffer.lookup("map", "map").unwrap(), Transform::default());
    }

    /// The pre-relocalize state on the 3D stack: pointlio_odom -> pointlio_body is being
    /// published, map -> pointlio_odom is not, so `map` has never been mentioned.
    #[test]
    fn an_unseen_frame_is_reported_as_unknown() {
        let mut buffer = TfBuffer::default();
        buffer
            .insert(
                "pointlio_odom",
                "pointlio_body",
                transform([1.0, 0.0, 0.0], 0.0),
            )
            .unwrap();

        assert_eq!(
            buffer.lookup("map", "pointlio_odom"),
            Err(LookupError::UnknownFrame("map".into()))
        );
    }

    #[test]
    fn two_separate_trees_do_not_connect() {
        let mut buffer = TfBuffer::default();
        buffer
            .insert("map", "odom", transform([0.0, 0.0, 0.0], 0.0))
            .unwrap();
        buffer
            .insert("other_map", "base_link", transform([0.0, 0.0, 0.0], 0.0))
            .unwrap();

        assert_eq!(
            buffer.lookup("map", "base_link"),
            Err(LookupError::NotConnected {
                target: "map".into(),
                source: "base_link".into(),
            })
        );
    }

    #[test]
    fn a_cycle_is_detected_rather_than_walked_forever() {
        let mut buffer = TfBuffer::default();
        buffer
            .insert("a", "b", transform([1.0, 0.0, 0.0], 0.0))
            .unwrap();
        buffer
            .insert("b", "a", transform([1.0, 0.0, 0.0], 0.0))
            .unwrap();

        assert_eq!(
            buffer.lookup("a", "b"),
            Err(LookupError::CycleDetected("a".into()))
        );
    }

    #[test]
    fn the_newest_transform_for_a_frame_replaces_the_previous_one() {
        let mut buffer = TfBuffer::default();
        buffer
            .insert("map", "base_link", transform([1.0, 0.0, 0.0], 0.0))
            .unwrap();
        buffer
            .insert("map", "base_link", transform([9.0, 0.0, 0.0], 0.0))
            .unwrap();

        assert_close(
            buffer.lookup("map", "base_link").unwrap().translation[0],
            9.0,
        );
    }

    #[test]
    fn malformed_transforms_are_rejected_and_change_nothing() {
        let mut buffer = TfBuffer::default();
        buffer
            .insert("map", "base_link", transform([1.0, 0.0, 0.0], 0.0))
            .unwrap();

        let zero_quaternion = Transform {
            translation: [7.0, 0.0, 0.0],
            rotation: Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 0.0,
            },
        };
        assert_eq!(
            buffer.insert("map", "base_link", zero_quaternion),
            Err(InvalidTransform::NotUnitQuaternion)
        );
        assert_eq!(
            buffer.insert("map", "map", transform([0.0, 0.0, 0.0], 0.0)),
            Err(InvalidTransform::SelfParent)
        );
        assert_eq!(
            buffer.insert("", "base_link", transform([0.0, 0.0, 0.0], 0.0)),
            Err(InvalidTransform::EmptyFrameId)
        );
        assert_eq!(
            buffer.insert("map", "base_link", transform([f64::NAN, 0.0, 0.0], 0.0)),
            Err(InvalidTransform::NonFinite)
        );

        // The good transform from the top is still the one that answers
        assert_close(
            buffer.lookup("map", "base_link").unwrap().translation[0],
            1.0,
        );
    }
}
