use rclrs::*;
use std::sync::Arc;
use std::time::Duration;

/// The range a rate parameter may turn into.
///
/// Both ends are guards the C++ version does not have: it divides `1.0 / rate` straight into a
/// `std::chrono::duration` and hands it to `create_wall_timer`, so a zero or negative rate there
/// is undefined at worst and a timer that never stops firing at best. `Duration::from_secs_f64`
/// panics instead of saturating, so the clamp has to happen first.
const MIN_PERIOD: Duration = Duration::from_millis(1);
const MAX_PERIOD: Duration = Duration::from_secs(3600);

/// The code default, and what the launch file passes too.
pub const DEFAULT_PUBLISH_RATE_HZ: f64 = 20.0;
/// The code default, and what the launch file passes too.
pub const DEFAULT_TRANSFORM_TOLERANCE_S: f64 = 0.1;

/// Hz -> timer period.
///
/// A non-positive rate would make the timer either fail or fire as fast as the executor allows, so
/// it is clamped to a slow but harmless 1 Hz rather than trusted. NaN and infinity take the same
/// path, and the resulting period is then held inside [`MIN_PERIOD`]..=[`MAX_PERIOD`].
#[must_use]
pub fn period_from_rate(rate_hz: f64) -> Duration {
    let safe_rate = if rate_hz > 0.0 && rate_hz.is_finite() {
        rate_hz
    } else {
        1.0
    };

    // A positive finite rate divides to a positive number or to +inf (for a subnormal), never to
    // NaN, and `clamp` maps +inf onto MAX_PERIOD. Clamped BEFORE the conversion: from_secs_f64
    // panics on a value that overflows a Duration, so clamping the Duration afterwards would be
    // too late.
    let seconds = 1.0 / safe_rate;
    Duration::from_secs_f64(seconds.clamp(MIN_PERIOD.as_secs_f64(), MAX_PERIOD.as_secs_f64()))
}

/// Seconds -> the `Duration` the TF stamps are future-dated by, or `None` when the value cannot
/// be one.
///
/// **Difference from the C++ version:** `rclcpp::Duration::from_seconds` accepts a negative
/// tolerance and back-dates the stamp; a `Duration` cannot be negative. A negative, NaN or
/// infinite value is therefore reported to the caller (who logs it) and replaced with 0 — the
/// stamp is then simply `now`, which is what a nav stack sees from any plain odometry source.
#[must_use]
pub fn tolerance_from_seconds(seconds: f64) -> Option<Duration> {
    if !(seconds >= 0.0 && seconds.is_finite()) {
        return None;
    }
    Some(Duration::from_secs_f64(
        seconds.min(MAX_PERIOD.as_secs_f64()),
    ))
}

/// The effective values the node runs on, after clamping and validation.
///
/// Held as plain owned values rather than read through the parameter handles on every tick: they
/// cannot change (see [`Parameters`]).
pub struct Config {
    /// Never namespaced: `map` is the one frame every robot shares.
    pub map_frame: String,
    /// TF frame names are not namespaced by ROS; the launch file prefixes these three with
    /// `<robot_id>/` itself.
    pub base_frame: String,
    pub odom_frame: String,
    /// Must name the URDF's lidar link (`lidar_top`), published by `syncai_bringup`. Without it
    /// the node publishes nothing at all — the extrinsic lookup gates the whole timer body.
    pub lidar_frame: String,
    pub publish_period: Duration,
    /// How far ahead of `now` every TF stamp is dated, the same trick AMCL uses: consumers can
    /// interpolate up to that far ahead without hitting an extrapolation error, which also
    /// bridges the gap between LIO's ~10 Hz updates and this node's 20 Hz republishing.
    pub transform_tolerance: Duration,
}

/// The node's parameters, all of them load-time only.
///
/// **Difference from the C++ version:** these are declared `read_only()`. The C++ node reads every
/// parameter once in its constructor and has no `add_on_set_parameters_callback`, so
/// `ros2 param set` there appears to succeed and silently changes nothing. Declaring them
/// read-only makes that same fact visible: the set is rejected instead of ignored. Values still
/// come from the launch file's inline parameters, which are applied at declaration time, before
/// the parameter is locked.
///
/// The handles are kept for the node's lifetime because a parameter is undeclared when its handle
/// drops.
pub struct Parameters {
    _map_frame: ReadOnlyParameter<Arc<str>>,
    _base_frame: ReadOnlyParameter<Arc<str>>,
    _odom_frame: ReadOnlyParameter<Arc<str>>,
    _lidar_frame: ReadOnlyParameter<Arc<str>>,
    _publish_rate: ReadOnlyParameter<f64>,
    _transform_tolerance: ReadOnlyParameter<f64>,
}

impl Parameters {
    pub fn declare(node: &Node) -> Result<(Self, Config), DeclarationError> {
        let text = |name: &str, default: &str, description: &str| {
            node.declare_parameter(name)
                // Turbofished: nothing downstream pins the element type, so a bare
                // Arc::from leaves the parameter as ReadOnlyParameter<Arc<_>>
                .default(Arc::<str>::from(default))
                .description(description)
                .read_only()
        };
        let number = |name: &str, default: f64, description: &str| {
            node.declare_parameter(name)
                .default(default)
                .description(description)
                .read_only()
        };

        let map_frame = text("map_frame", "map", "the global frame; never namespaced")?;
        let base_frame = text(
            "base_frame",
            "base_link",
            "the robot base frame; the launch file prefixes it with <robot_id>/",
        )?;
        let odom_frame = text(
            "odom_frame",
            "odom",
            "the odom frame this node publishes; the launch file prefixes it with <robot_id>/",
        )?;
        let lidar_frame = text(
            "lidar_frame",
            "lidar_top",
            "the URDF lidar link Point-LIO's body frame physically is; must exist in TF",
        )?;
        let publish_rate = number(
            "publish_rate",
            DEFAULT_PUBLISH_RATE_HZ,
            "rate of the TF broadcasts and the odom topic, in Hz",
        )?;
        let transform_tolerance = number(
            "transform_tolerance",
            DEFAULT_TRANSFORM_TOLERANCE_S,
            "seconds the broadcast TF stamps are future-dated by",
        )?;

        let tolerance = tolerance_from_seconds(transform_tolerance.get()).unwrap_or_else(|| {
            log_error!(
                node.logger(),
                "[Parameters] transform_tolerance must be a finite, non-negative number of \
                 seconds, got {}; stamping TF at `now` instead",
                transform_tolerance.get(),
            );
            Duration::ZERO
        });

        let config = Config {
            map_frame: map_frame.get().to_string(),
            base_frame: base_frame.get().to_string(),
            odom_frame: odom_frame.get().to_string(),
            lidar_frame: lidar_frame.get().to_string(),
            publish_period: period_from_rate(publish_rate.get()),
            transform_tolerance: tolerance,
        };

        log_info!(
            node.logger(),
            "[Parameters] frames: map '{}', odom '{}', base '{}', lidar '{}'",
            config.map_frame,
            config.odom_frame,
            config.base_frame,
            config.lidar_frame,
        );
        log_info!(
            node.logger(),
            "[Parameters] publish every {:?}, TF stamps future-dated by {:?}",
            config.publish_period,
            config.transform_tolerance,
        );

        Ok((
            Self {
                _map_frame: map_frame,
                _base_frame: base_frame,
                _odom_frame: odom_frame,
                _lidar_frame: lidar_frame,
                _publish_rate: publish_rate,
                _transform_tolerance: transform_tolerance,
            },
            config,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_rate_becomes_its_period() {
        assert_eq!(period_from_rate(1.0), Duration::from_secs(1));
        assert_eq!(period_from_rate(20.0), Duration::from_millis(50));
        assert_eq!(period_from_rate(2.0), Duration::from_millis(500));
    }

    #[test]
    fn a_non_positive_rate_is_clamped_to_one_hz_rather_than_trusted() {
        assert_eq!(period_from_rate(0.0), Duration::from_secs(1));
        assert_eq!(period_from_rate(-5.0), Duration::from_secs(1));
    }

    /// `Duration::from_secs_f64` panics on NaN and on the infinity a denormal rate divides to, so
    /// a hostile params file must not reach it.
    #[test]
    fn unusable_rates_do_not_panic() {
        assert_eq!(period_from_rate(f64::NAN), Duration::from_secs(1));
        assert_eq!(period_from_rate(f64::INFINITY), Duration::from_secs(1));
        assert_eq!(period_from_rate(f64::NEG_INFINITY), Duration::from_secs(1));
        assert_eq!(period_from_rate(f64::MIN_POSITIVE), MAX_PERIOD);
        // The smallest subnormal divides to +inf, which must clamp like any other huge period
        assert_eq!(period_from_rate(5e-324), MAX_PERIOD);
        // A rate so high the period rounds to zero would give a timer that never stops firing
        assert_eq!(period_from_rate(1e9), MIN_PERIOD);
    }

    #[test]
    fn a_tolerance_becomes_its_duration() {
        assert_eq!(
            tolerance_from_seconds(0.1),
            Some(Duration::from_millis(100))
        );
        assert_eq!(tolerance_from_seconds(0.0), Some(Duration::ZERO));
        assert_eq!(tolerance_from_seconds(1e12), Some(MAX_PERIOD));
    }

    #[test]
    fn an_unusable_tolerance_is_reported_rather_than_panicking() {
        assert_eq!(tolerance_from_seconds(-0.1), None);
        assert_eq!(tolerance_from_seconds(f64::NAN), None);
        assert_eq!(tolerance_from_seconds(f64::INFINITY), None);
        assert_eq!(tolerance_from_seconds(f64::NEG_INFINITY), None);
    }
}
