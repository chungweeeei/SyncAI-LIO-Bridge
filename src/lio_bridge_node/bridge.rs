//! The math of the bridge — the body of the C++ `timer_cb()` with the ROS I/O taken out.
//!
//! Pure functions over [`Transform`] and [`Pose2D`], so the two formulas the README documents are
//! unit-tested here against hand-computable poses, without a ROS environment or the robot.
//!
//! ```text
//! odom -> base  =  (lio_odom -> lio_body) · (base_link -> lidar_top_imu)⁻¹    then P2D
//! map  -> base  =  (map -> lio_odom) · (lio_odom -> base)                      (unprojected)
//! map  -> odom  =  P2D(map -> base) · P2D(odom -> base)⁻¹
//! ```

use super::geometry::{Pose2D, Transform};

/// The planar odom chain and the 3D transform it was projected from.
///
/// Both are needed downstream: the projection is what gets broadcast as `odom -> base_link` and
/// subtracted in [`map_to_odom`], while the unprojected `lio_odom -> base` is what the map
/// correction has to be composed with BEFORE projecting (see that function).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OdomChain {
    /// `lio_odom -> base`, full 3D.
    pub lio_odom_base: Transform,
    /// Its planar projection — the `odom -> base_link` that is broadcast.
    pub odom_base: Pose2D,
}

/// `odom -> base_link` from the LIO pose and the lidar mount extrinsic.
///
/// Point-LIO's "body" frame is physically the lidar's built-in IMU, so the static
/// `base_link -> lidar_top_imu` extrinsic (the lidar mount plus the IMU's offset inside the
/// lidar, from the URDF) maps it back to the robot base: `lio_odom -> base = lio_odom -> lio_body
/// · lidar -> base`, where `lidar` is that IMU link. The LIO odom frame is then taken to BE the
/// odom frame — nothing else publishes one.
pub fn odom_to_base(lio_pose: Transform, base_lidar: Transform) -> OdomChain {
    let lio_odom_base = lio_pose.then(base_lidar.inverse());
    OdomChain {
        lio_odom_base,
        odom_base: Pose2D::project(lio_odom_base),
    }
}

/// The AMCL-style `map -> odom` correction: the map-frame pose minus the odom-frame pose.
///
/// `map_lio_odom` is the localizer's `map -> <lio odom frame>`. The first line composes the
/// **unprojected** LIO transforms, so `map -> base` is the full 3D pose; it is projected only
/// once, in the second line, where both operands of the subtraction are planar. That keeps the
/// result planar and consistent with the `odom -> base_link` that was just broadcast, without
/// throwing away the tilt before the map-frame composition: projecting `lio_odom -> base` first
/// and then multiplying by a tilted `map -> lio_odom` would not give the same planar pose.
pub fn map_to_odom(map_lio_odom: Transform, chain: OdomChain) -> Pose2D {
    let map_base = Pose2D::project(map_lio_odom.then(chain.lio_odom_base));
    Pose2D::project(
        map_base
            .to_transform()
            .then(chain.odom_base.to_transform().inverse()),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::lio_bridge_node::geometry::Quaternion;
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_4};

    fn assert_close(actual: f64, expected: f64) {
        assert!(
            (actual - expected).abs() < 1e-9,
            "expected {expected}, got {actual}"
        );
    }

    fn assert_pose_close(actual: Pose2D, expected: Pose2D) {
        assert_close(actual.x, expected.x);
        assert_close(actual.y, expected.y);
        assert_close(actual.yaw, expected.yaw);
    }

    fn planar(x: f64, y: f64, yaw: f64) -> Transform {
        Pose2D { x, y, yaw }.to_transform()
    }

    fn pitch(angle: f64) -> Quaternion {
        Quaternion {
            x: 0.0,
            y: (angle / 2.0).sin(),
            z: 0.0,
            w: (angle / 2.0).cos(),
        }
    }

    /// The robot's actual mount: lidar_top 0.2 m ahead of and 0.5 m above base_link, pitched
    /// 0.25 rad.
    fn lidar_mount() -> Transform {
        Transform {
            translation: [0.2, 0.0, 0.5],
            rotation: pitch(0.25),
        }
    }

    #[test]
    fn with_the_lidar_at_the_base_the_odom_pose_is_the_lio_pose_projected() {
        let lio_pose = Transform {
            translation: [1.0, 2.0, 0.3],
            rotation: Quaternion::from_yaw(FRAC_PI_4).multiply(pitch(0.1)),
        };
        let chain = odom_to_base(lio_pose, Transform::default());
        assert_eq!(chain.lio_odom_base, lio_pose);
        assert_pose_close(
            chain.odom_base,
            Pose2D {
                x: 1.0,
                y: 2.0,
                yaw: FRAC_PI_4,
            },
        );
    }

    /// With the lidar sitting at the LIO origin, the base must come out behind and below it, by
    /// exactly the mount offset, and level.
    #[test]
    fn the_mount_extrinsic_is_undone() {
        let chain = odom_to_base(Transform::default(), lidar_mount());
        // lidar -> base is the inverse mount; its yaw is 0 and its z is irrelevant to the plane
        let expected = lidar_mount().inverse();
        assert_close(chain.odom_base.x, expected.translation[0]);
        assert_close(chain.odom_base.y, expected.translation[1]);
        assert_close(chain.odom_base.yaw, 0.0);
        // and the full 3D transform is kept for the map composition
        assert_eq!(chain.lio_odom_base, expected);
    }

    /// A lidar yawed 90 degrees relative to the base: LIO reporting "facing +x" means the base
    /// faces -y, and the base sits off to the side of where the lidar is.
    #[test]
    fn a_yawed_mount_turns_the_base() {
        let base_lidar = planar(0.0, 0.3, FRAC_PI_2);
        let lio_pose = planar(5.0, 5.0, 0.0);
        let chain = odom_to_base(lio_pose, base_lidar);
        // base -> lidar is +0.3 along base y with a quarter turn; so lidar -> base is a quarter
        // turn back and the base lies at lidar-frame (-0.3, 0): i.e. 0.3 m behind the lidar's
        // heading, which in lio_odom is at (4.7, 5.0)
        assert_pose_close(
            chain.odom_base,
            Pose2D {
                x: 4.7,
                y: 5.0,
                yaw: -FRAC_PI_2,
            },
        );
    }

    #[test]
    fn an_identity_correction_gives_an_identity_map_to_odom() {
        let chain = odom_to_base(planar(3.0, -1.0, 0.8), lidar_mount());
        let mo = map_to_odom(Transform::default(), chain);
        assert_pose_close(mo, Pose2D::default());
    }

    /// The invariant AMCL's correction satisfies and the nav stack relies on: composing the
    /// broadcast `map -> odom` with the broadcast `odom -> base_link` reproduces the planar
    /// `map -> base`.
    #[test]
    fn map_to_odom_composed_with_odom_to_base_is_the_planar_map_pose() {
        let map_lio_odom = Transform {
            translation: [10.0, -4.0, 0.2],
            rotation: Quaternion::from_yaw(1.1).multiply(pitch(0.05)),
        };
        let lio_pose = Transform {
            translation: [2.0, 1.5, 0.1],
            rotation: Quaternion::from_yaw(-0.6).multiply(pitch(-0.12)),
        };
        let chain = odom_to_base(lio_pose, lidar_mount());
        let mo = map_to_odom(map_lio_odom, chain);

        let expected = Pose2D::project(map_lio_odom.then(chain.lio_odom_base));
        let actual = Pose2D::project(mo.to_transform().then(chain.odom_base.to_transform()));
        assert_pose_close(actual, expected);
    }

    /// A pure planar case that can be checked by hand: map -> lio_odom is a quarter turn at
    /// (1, 2); the lidar is at the base; LIO says the base is 1 m along lio_odom's x. Then
    /// map -> base is (1, 3) facing +y, odom -> base is (1, 0) facing +x, and the correction is
    /// the quarter turn at (1, 2) itself.
    #[test]
    fn a_planar_correction_is_the_localizer_transform() {
        let map_lio_odom = planar(1.0, 2.0, FRAC_PI_2);
        let chain = odom_to_base(planar(1.0, 0.0, 0.0), Transform::default());
        assert_pose_close(
            map_to_odom(map_lio_odom, chain),
            Pose2D {
                x: 1.0,
                y: 2.0,
                yaw: FRAC_PI_2,
            },
        );
    }

    /// The point of composing before projecting: with a tilted `map -> lio_odom`, projecting
    /// `lio_odom -> base` first gives a different planar pose.
    #[test]
    fn the_map_pose_is_composed_in_3d_before_projecting() {
        let map_lio_odom = Transform {
            translation: [0.0, 0.0, 0.0],
            rotation: pitch(0.4),
        };
        let lio_pose = Transform {
            translation: [0.0, 0.0, 2.0],
            rotation: Quaternion::default(),
        };
        let chain = odom_to_base(lio_pose, Transform::default());

        // Correct: a point 2 m up, seen through a 0.4 rad pitch, moves along x in the map
        let correct = Pose2D::project(map_lio_odom.then(chain.lio_odom_base));
        assert_close(correct.x, 2.0 * (0.4_f64).sin());

        // Projecting first would have thrown the 2 m of z away and left x at 0
        let projected_first = Pose2D::project(map_lio_odom.then(chain.odom_base.to_transform()));
        assert_close(projected_first.x, 0.0);

        // and map_to_odom uses the correct one
        let mo = map_to_odom(map_lio_odom, chain);
        assert_close(mo.x, 2.0 * (0.4_f64).sin());
    }
}
