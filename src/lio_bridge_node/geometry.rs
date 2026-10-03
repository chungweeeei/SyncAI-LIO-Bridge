//! The rigid-transform arithmetic the C++ node gets from `tf2::Transform`, `tf2::Quaternion` and
//! `tf2::Matrix3x3`, plus the planar [`Pose2D`] projection every broadcast goes through.
//!
//! Pure: no ROS types, no clock, no logging. The subscriptions convert `geometry_msgs` into these
//! at the boundary, so everything here (and in `tf.rs` / `bridge.rs`, which build on it) is
//! unit-tested without a ROS environment or the robot.

/// A quaternion in the x/y/z/w order `geometry_msgs/Quaternion` uses.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

impl Default for Quaternion {
    /// The identity rotation — NOT all zeroes, which is not a rotation at all.
    fn default() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0,
        }
    }
}

impl Quaternion {
    /// A rotation of `yaw` radians about Z, i.e. `tf2::Quaternion::setRPY(0, 0, yaw)`.
    pub fn from_yaw(yaw: f64) -> Self {
        let (sin, cos) = (yaw / 2.0).sin_cos();
        Self {
            x: 0.0,
            y: 0.0,
            z: sin,
            w: cos,
        }
    }

    /// Hamilton product, i.e. "apply `rhs` first, then `self`".
    pub fn multiply(self, rhs: Self) -> Self {
        Self {
            x: self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            y: self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            z: self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
            w: self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
        }
    }

    /// The inverse rotation. Only correct for a unit quaternion, which `TfBuffer::insert` and the
    /// LIO intake both check for before anything reaches here.
    pub fn conjugate(self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
            z: -self.z,
            w: self.w,
        }
    }

    pub fn rotate(self, v: [f64; 3]) -> [f64; 3] {
        // q * (0, v) * q^-1, expanded so no temporary quaternion is built
        let u = [self.x, self.y, self.z];
        let uv = cross(u, v);
        let uuv = cross(u, uv);
        [
            v[0] + 2.0 * (self.w * uv[0] + uuv[0]),
            v[1] + 2.0 * (self.w * uv[1] + uuv[1]),
            v[2] + 2.0 * (self.w * uv[2] + uuv[2]),
        ]
    }

    pub fn norm(self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w).sqrt()
    }

    pub fn is_finite(self) -> bool {
        [self.x, self.y, self.z, self.w]
            .iter()
            .all(|v| v.is_finite())
    }

    /// `atan2(R[1][0], R[0][0])` of the rotation matrix this quaternion describes — the yaw the
    /// C++ `project_2d()` reads straight off `tf2::Matrix3x3`, and the Python node's before it.
    ///
    /// Deliberately NOT `tf2::getYaw`, which has a degenerate branch at +/-90 degrees of pitch
    /// that reports a yaw of 0. The C++ node never used it, so reproducing it would make this a
    /// different node, not a port.
    pub fn yaw(self) -> f64 {
        let m10 = 2.0 * (self.x * self.y + self.w * self.z);
        let m00 = 1.0 - 2.0 * (self.y * self.y + self.z * self.z);
        m10.atan2(m00)
    }
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// A rigid transform, read as `parent <- child`: the child frame expressed in the parent's
/// coordinates. That is exactly what one `geometry_msgs/TransformStamped` carries (with
/// `header.frame_id` the parent and `child_frame_id` the child) and what a `geometry_msgs/Pose`
/// in a stamped message carries relative to its `header.frame_id`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Transform {
    pub translation: [f64; 3],
    pub rotation: Quaternion,
}

impl Transform {
    /// Compose: `self` is `A <- B` and `rhs` is `B <- C`, so the result is `A <- C`. This is
    /// `tf2::Transform::operator*` with the operands in the same order.
    pub fn then(self, rhs: Self) -> Self {
        let t = self.rotation.rotate(rhs.translation);
        Self {
            translation: [
                self.translation[0] + t[0],
                self.translation[1] + t[1],
                self.translation[2] + t[2],
            ],
            rotation: self.rotation.multiply(rhs.rotation),
        }
    }

    /// `A <- B` becomes `B <- A`.
    pub fn inverse(self) -> Self {
        let rotation = self.rotation.conjugate();
        let t = rotation.rotate(self.translation);
        Self {
            translation: [-t[0], -t[1], -t[2]],
            rotation,
        }
    }

    pub fn is_finite(self) -> bool {
        self.translation.iter().all(|v| v.is_finite()) && self.rotation.is_finite()
    }
}

/// Planar (x, y, yaw) projection of a rigid transform: `x`/`y` taken directly, `yaw` from the
/// rotation matrix, and `z`, roll and pitch dropped.
///
/// Every broadcast goes through this so the planar nav stack never sees a tilted frame — which
/// matters more here than on a wheeled robot, because the quadruped's body pitches and rolls with
/// every gait cycle and the lidar itself is mounted at a 0.25 rad tilt. A costmap fed a tilted
/// `base_link` would smear obstacles.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Pose2D {
    pub x: f64,
    pub y: f64,
    /// Radians.
    pub yaw: f64,
}

impl Pose2D {
    /// The C++ `project_2d()`.
    pub fn project(t: Transform) -> Self {
        Self {
            x: t.translation[0],
            y: t.translation[1],
            yaw: t.rotation.yaw(),
        }
    }

    /// The C++ `pose_2d_transform()`: back to a rigid transform with z / roll / pitch zero.
    pub fn to_transform(self) -> Transform {
        Transform {
            translation: [self.x, self.y, 0.0],
            rotation: Quaternion::from_yaw(self.yaw),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI};

    fn assert_close(actual: f64, expected: f64) {
        assert!(
            (actual - expected).abs() < 1e-9,
            "expected {expected}, got {actual}"
        );
    }

    fn assert_transform_close(actual: Transform, expected: Transform) {
        for i in 0..3 {
            assert_close(actual.translation[i], expected.translation[i]);
        }
        // q and -q are the same rotation, so compare the rotations by what they do
        let probe = [0.3, -0.7, 1.1];
        let a = actual.rotation.rotate(probe);
        let e = expected.rotation.rotate(probe);
        for i in 0..3 {
            assert_close(a[i], e[i]);
        }
    }

    #[test]
    fn yaw_round_trips_through_the_quaternion() {
        for yaw in [0.0, 0.5, -0.5, 1.5, -1.5, 3.0, -3.0] {
            assert_close(Quaternion::from_yaw(yaw).yaw(), yaw);
        }
    }

    #[test]
    fn identity_quaternion_has_zero_yaw() {
        assert_close(Quaternion::default().yaw(), 0.0);
        assert_close(Quaternion::default().norm(), 1.0);
    }

    /// The reason the projection exists: a pitched body must still report its heading. Pitch by
    /// 0.25 rad (the lidar mount tilt) after a yaw of 45 degrees and the yaw must survive.
    #[test]
    fn yaw_survives_a_pitch() {
        let pitch = Quaternion {
            x: 0.0,
            y: (0.25_f64 / 2.0).sin(),
            z: 0.0,
            w: (0.25_f64 / 2.0).cos(),
        };
        let yawed_then_pitched = Quaternion::from_yaw(FRAC_PI_4).multiply(pitch);
        assert_close(yawed_then_pitched.yaw(), FRAC_PI_4);
    }

    #[test]
    fn rotating_x_by_a_quarter_turn_gives_y() {
        let v = Quaternion::from_yaw(FRAC_PI_2).rotate([1.0, 0.0, 0.0]);
        assert_close(v[0], 0.0);
        assert_close(v[1], 1.0);
        assert_close(v[2], 0.0);
    }

    #[test]
    fn composition_applies_the_right_operand_first() {
        // A <- B: 2 m along x, then a quarter turn. B <- C: 1 m along B's x.
        let a_b = Transform {
            translation: [2.0, 0.0, 0.0],
            rotation: Quaternion::from_yaw(FRAC_PI_2),
        };
        let b_c = Transform {
            translation: [1.0, 0.0, 0.0],
            rotation: Quaternion::default(),
        };
        // C's origin sits 1 m along B's x, which A sees as +y
        let a_c = a_b.then(b_c);
        assert_close(a_c.translation[0], 2.0);
        assert_close(a_c.translation[1], 1.0);
        assert_close(a_c.rotation.yaw(), FRAC_PI_2);
    }

    #[test]
    fn a_transform_composed_with_its_inverse_is_the_identity() {
        let t = Transform {
            translation: [1.0, -2.0, 0.5],
            rotation: Quaternion::from_yaw(1.2).multiply(Quaternion {
                x: (0.3_f64).sin(),
                y: 0.0,
                z: 0.0,
                w: (0.3_f64).cos(),
            }),
        };
        assert_transform_close(t.then(t.inverse()), Transform::default());
        assert_transform_close(t.inverse().then(t), Transform::default());
    }

    #[test]
    fn projecting_drops_z_roll_and_pitch_and_keeps_x_y_yaw() {
        let tilted = Transform {
            translation: [3.0, 4.0, 0.7],
            rotation: Quaternion::from_yaw(PI / 3.0).multiply(Quaternion {
                x: 0.0,
                y: (0.1_f64).sin(),
                z: 0.0,
                w: (0.1_f64).cos(),
            }),
        };
        let p = Pose2D::project(tilted);
        assert_close(p.x, 3.0);
        assert_close(p.y, 4.0);
        assert_close(p.yaw, PI / 3.0);

        let back = p.to_transform();
        assert_close(back.translation[2], 0.0);
        assert_close(back.rotation.x, 0.0);
        assert_close(back.rotation.y, 0.0);
        assert_close(back.rotation.yaw(), PI / 3.0);
    }

    #[test]
    fn non_finite_values_are_detected() {
        assert!(Transform::default().is_finite());
        let mut t = Transform::default();
        t.translation[1] = f64::NAN;
        assert!(!t.is_finite());
        let mut t = Transform::default();
        t.rotation.w = f64::INFINITY;
        assert!(!t.is_finite());
    }
}
