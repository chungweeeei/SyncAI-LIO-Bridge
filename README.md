# syncai_lio_bridge (Rust)

The ROS 2 package `syncai_lio_bridge`, written with [`ros2_rust`](https://github.com/ros2-rust/ros2_rust)
(`rclrs` 0.8), plus a Docker environment for developing it on its own.

It is a Rust port of the C++ (rclcpp) `syncai_lio_bridge` in SyncAI-Robot-Workspace, and **its
external interface is deliberately identical**: the node name `lio_bridge_node`, the executable
`lio_bridge_node`, the launch file `lio_bridge.launch.py`, parameter names, topics, frames,
message types and QoS are all the same, so it is a drop-in replacement and nothing else in the
stack changes. For behavioural detail the C++ README is authoritative; [Differences from the C++
version](#differences-from-the-c-version) below lists every place this port knowingly deviates.

**This repo is itself a single colcon package** (`package.xml` / `Cargo.toml` live at the root).
Like SyncAI-Robot-State and SyncAI-Robot-Driver-Manager, whose layout and conventions it follows,
it is pulled into SyncAI-Robot-Workspace with vcstool as `src/syncai_lio_bridge`.

## The node

One node, `lio_bridge_node`, that turns the FAST-LIO2 / Point-LIO estimate into the planar TF
chain the nav stack expects. It is **the robot's only odometry source and its localization
provider** — it replaces wheel odometry *and* AMCL at the same time.

```
  localizer            pointlio                      URDF (robot_state_publisher)
      │                    │                                    │
   map ──► <id>/pointlio_odom ──► <id>/pointlio_body  <id>/base_link ──► <id>/lidar_top_imu
      (TF, after relocalize)   (TF + pointlio/lio_odom)   (static, via <id>/lidar_top: the
                                                           lidar_top and lidar_top_imu joints)
                                        │
                                        ▼
                               ┌─────────────────┐
                    livox/imu ─►   lio_bridge    │
                                └────────┬────────┘
                    ┌────────────────────┼────────────────────┐
                    ▼                    ▼                    ▼
          map ──► <id>/odom     <id>/odom ──► <id>/base_link   /<id>/odom
        (TF, AMCL replacement)        (TF, the odom chain)   (nav_msgs/Odometry)
```

Wheel odometry is gone — the Isaac Sim OmniGraph odom publishers are deliberately disabled, and
the real chassis publishes none. Nothing else in the stack broadcasts `odom → base_link` or
`map → odom`.

### Inputs

| Input | Kind | Source | Used for |
|---|---|---|---|
| `pointlio/lio_odom` | `nav_msgs/Odometry`, SensorData QoS | Point-LIO | Pose (`lio_odom → lio_body`) and body-frame linear velocity |
| `livox/imu` | `sensor_msgs/Imu`, SensorData QoS | Livox MID360 driver | Yaw rate |
| `base_frame → lidar_frame` | TF (static, via `/tf_static`) | `syncai_bringup`'s URDF | The lidar mount extrinsic |
| `map_frame → <lio_odom frame>` | TF (via `/tf`) | `syncai_localizer`, **only after relocalize** | The map correction |

Both sensor subscriptions use SensorData (best-effort) QoS, which is compatible with either a
reliable or a best-effort publisher. The TF subscriptions use tf2's own listener QoS (reliable,
depth 100, `/tf_static` transient-local).

The LIO odom frame is **not** a parameter: it is read from the incoming message's
`header.frame_id`. Point-LIO's `world_frame` is configured per-setup (`pointlio_odom` on the Isaac
config, `lio_odom` upstream), so taking it from the header means a rename upstream needs no change
here.

### Outputs

| Output | Type | QoS |
|---|---|---|
| `<odom_frame> → <base_frame>` | TF broadcast on `/tf` | reliable, depth 100 |
| `<map_frame> → <odom_frame>` | TF broadcast on `/tf` | reliable, depth 100 |
| `odom` | `nav_msgs/Odometry` | reliable, depth 10 |

All three are produced by a single timer at `publish_rate` (20 Hz by default), independent of the
~10 Hz rate at which LIO actually updates — the timer republishes the latest cached state.

### The math

**odom → base_link.** Point-LIO's "body" frame is physically the lidar's built-in IMU, so the
static extrinsic to that IMU — the lidar mount plus the IMU's offset inside the lidar, both from
the URDF — maps it back to the robot base:

```
odom→base  =  (lio_odom→lio_body) · (base_link→lidar_top_imu)⁻¹
```

then projected to 2D. The static extrinsic is looked up once and cached; until it resolves, the
node logs a throttled "waiting for base_link -> lidar_top_imu" and publishes nothing.

**map → odom.** The AMCL-style correction, computed exactly as AMCL does — the map-frame pose
minus the odom-frame pose:

```
map→base  =  (map→lio_odom) · (lio_odom→base)
map→odom  =  P2D(map→base) · P2D(odom→base)⁻¹
```

The first line composes the **unprojected** LIO transforms, so `map→base` is the full 3D pose; it
is projected only once, in the second line, where both operands of the subtraction are 2D. That
keeps the result planar and consistent with the `odom → base_link` that was just broadcast,
without throwing away the tilt before the map-frame composition. Both formulas live in
`src/lio_bridge_node/bridge.rs` and are unit-tested there, including the invariant that
`map→odom · odom→base` reproduces the planar `map→base`.

### Everything is projected to 2D

`Pose2D::project()` keeps `x`, `y` and `yaw = atan2(R[1][0], R[0][0])`, and zeroes `z`, roll and
pitch. This happens before every broadcast so the planar nav stack never sees a tilted frame —
which matters here more than on a wheeled robot, because the quadruped's body pitches and rolls
with every gait cycle and the lidar itself is mounted at a 0.25 rad tilt. A costmap fed a tilted
`base_link` would smear obstacles.

### Angular velocity comes from the IMU

`odom.twist.twist.angular.z` is taken from the lidar IMU's gyro `z`. The IMU is co-located with
the lidar and the robot is treated as planar, so gyro z is the yaw rate. Linear x/y come from
LIO's own body-frame twist.

This is a deliberate choice, not a fallback: `pointlio_node` does fill `twist.angular` from its
EKF output-model state, but with `gyr_cov_output: 1000.0` that estimate tracks the raw measurement
almost instantly and was measured 5.9x noisier than the gyro at standstill (robot01, 2026-09-21).
The twist is what consumers actually use: `syncai_controller`, `syncai_task_runner`'s odom
smoother and `syncai_robot_state`. None of them read the pose out of this topic.

### Two-stage startup

The node deliberately does **not** wait for localization before publishing the odom chain,
mirroring AMCL, where `odom → base_link` exists before an initial pose is given:

1. As soon as `pointlio/lio_odom` and the lidar extrinsic are available, it broadcasts
   `odom → base_link` and publishes the `odom` topic. The robot can be driven; the costmaps'
   `odom`-frame local costmap works.
2. `map → odom` additionally requires `map → <lio_odom frame>` from the localizer, which **only
   exists after `/<robot_id>/relocalize` has been called** (or the INI's `[initial_pose]` / an
   `initialpose` message has done the same). Until then the timer logs a throttled "waiting for TF
   (relocalized yet?)" and returns early. The first successful correction logs
   `localization bridged: map -> <id>/odom = (x, y, yaw …)` exactly once — that line is the
   definitive "the robot is localized" signal.

### Parameters

There is no params YAML; the launch file passes everything inline. All are **load-time only** and
declared read-only, so `ros2 param set` is rejected rather than silently ignored.

| Parameter | Default (node) | Launch value | Notes |
|---|---|---|---|
| `map_frame` | `map` | `map` | Never namespaced |
| `base_frame` | `base_link` | `<robot_id>/base_link` | |
| `odom_frame` | `odom` | `<robot_id>/odom` | |
| `lidar_frame` | `lidar_top_imu` | `<robot_id>/lidar_top_imu` | Must match the URDF's `lidar_top_imu` link (the lidar's IMU); `lidar_top` until 2026-10 |
| `publish_rate` | `20.0` | `20.0` | Hz; a non-positive or non-finite value falls back to 1 Hz |
| `transform_tolerance` | `0.1` | `0.1` | Seconds the TF stamp is future-dated; negative / NaN becomes 0 |
| `use_sim_time` | — | **`false`** | No `/clock` on the real robot; pass `use_sim_time:=true` only against a simulator |

`transform_tolerance` future-dates the broadcast stamp, the same trick AMCL uses: consumers can
interpolate up to that far ahead without hitting an extrapolation error, which also bridges the
gap between LIO's ~10 Hz updates and this node's 20 Hz republishing.

Frame names are given the `<robot_id>/` prefix explicitly by the launch file because ROS
namespaces topics but not TF frame ids; `map` stays plain.

### Diagnosing it

```bash
ros2 topic hz /<robot_id>/odom
ros2 run tf2_ros tf2_echo map <robot_id>/odom          # zero-ish until relocalize
ros2 run tf2_ros tf2_echo <robot_id>/odom <robot_id>/base_link
ros2 topic echo /<robot_id>/pointlio/lio_odom --once   # is LIO producing at all?
```

The node's own log is the fastest diagnosis — each failure mode has a distinct throttled message:

| Log line | Missing |
|---|---|
| `waiting for pointlio/lio_odom (LIO initializing?)` | Point-LIO is not publishing |
| `waiting for <id>/base_link -> <id>/lidar_top_imu` | `bringup` / `robot_state_publisher` is not running, or its URDF predates the `lidar_top_imu` link (SyncAI-Robot-Workspace before PR #44) |
| `waiting for TF (relocalized yet?)` | `/<robot_id>/relocalize` has not been called (and no `[initial_pose]` / `initialpose` has stood in for it) |

## Why Docker

ROS 2 has no official macOS support, and building ROS 2 + `rclrs` natively on macOS tends to get
stuck on dependencies. So the whole environment is packaged into a Linux container: **edit code on
the host, build and run inside the container.** The image carries ROS 2 Humble, Rust 1.85,
`colcon-cargo` / `colcon-ros-cargo`, and `rclrs` built from source into the underlay
`/opt/ros2_rust_underlay`, which is sourced automatically on entering the container.

## Quick start (VS Code Dev Container)

The only development environment is the Dev Container (`.devcontainer/`); there is no Makefile or
docker-compose. Once the package is in SyncAI-Robot-Workspace it is built with the Workspace's own
image, so this setup is only for developing this repo on its own.

1. Install the VS Code [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension
2. Open this repo in VS Code and run **Dev Containers: Reopen in Container**
3. The first run builds the image (slow — message packages are compiled from source); later runs
   open instantly. After creation, `postCreateCommand` runs `colcon build` once
4. Run the node from a VS Code terminal (the namespace comes from `robot_id` in
   `~/robot_ws/config/system.ini`, falling back to `default_robot`):

```bash
ros2 launch syncai_lio_bridge lio_bridge.launch.py
```

Without VS Code, use the [devcontainer CLI](https://github.com/devcontainers/cli):
`devcontainer up --workspace-folder .`, then `docker exec -it syncai-lio-bridge bash -l`.

The container is named `syncai-lio-bridge` and its volumes are prefixed the same way, so it runs
side by side with the SyncAI-Robot-State and SyncAI-Robot-Driver-Manager containers. All default
to `ROS_DOMAIN_ID=2` on the host network, so the nodes see each other's topics.

### Common commands (inside the container)

```bash
cd /workspace && colcon build --symlink-install --packages-select syncai_lio_bridge
cd /workspace/src/syncai_lio_bridge
cargo fmt                                                  # format (CI uses --check)
cargo clippy --target-dir /workspace/build/.clippy --all-targets
cargo test --target-dir /workspace/build/.clippy
```

`cargo` needs one `colcon build` first: `/workspace/.cargo/config.toml` is generated by
colcon-ros-cargo. Warnings from `/opt/ros2_rust_underlay/...` are rclrs's own; ignore them.

### CI

`.github/workflows/ci.yml` runs on every push to `main` / `dev` and on every pull request. One job
runs `cargo fmt --check` on the bare runner; the other builds this repo's Dev Container image with
buildx (layers cached between runs, so only the first run compiles rclrs from source), then runs
`.github/scripts/ci.sh` inside it: `colcon build`, `cargo clippy -- -D warnings` and `cargo test`.
The script can be run by hand inside the Dev Container to reproduce CI exactly.

### Running it without the robot

Use a separate `ROS_DOMAIN_ID` so the fakes can never reach the real robot:

```bash
export ROS_DOMAIN_ID=77
ros2 run syncai_lio_bridge lio_bridge_node --ros-args -r __ns:=/default_robot \
    -p base_frame:=default_robot/base_link -p odom_frame:=default_robot/odom \
    -p lidar_frame:=default_robot/lidar_top_imu
```

Expect `waiting for pointlio/lio_odom (LIO initializing?)` every 5 s. Then, in other shells (same
`ROS_DOMAIN_ID`):

```bash
# The lidar IMU's mount: 0.2 m ahead, 0.5 m up, pitched 0.25 rad (static, so transient-local;
# one link standing in for the URDF's base_link -> lidar_top -> lidar_top_imu)
ros2 run tf2_ros static_transform_publisher --x 0.2 --z 0.5 --pitch 0.25 \
    --frame-id default_robot/base_link --child-frame-id default_robot/lidar_top_imu

# LIO says the lidar is 1 m along pointlio_odom's x, moving at 0.5 m/s
ros2 topic pub -r 10 /default_robot/pointlio/lio_odom nav_msgs/msg/Odometry \
  "{header: {frame_id: default_robot/pointlio_odom}, child_frame_id: default_robot/pointlio_body, \
    pose: {pose: {position: {x: 1.0}, orientation: {w: 1.0}}}, twist: {twist: {linear: {x: 0.5}}}}"

ros2 run tf2_ros tf2_echo default_robot/odom default_robot/base_link   # stage 1: the odom chain
ros2 topic echo /default_robot/odom --once

# The localizer's correction: map -> pointlio_odom at (10, 20), a quarter turn
ros2 run tf2_ros static_transform_publisher --x 10 --y 20 --yaw 1.5707963 \
    --frame-id map --child-frame-id default_robot/pointlio_odom

ros2 run tf2_ros tf2_echo map default_robot/odom                       # stage 2: localized
```

The node logs `localization bridged: map -> default_robot/odom = (10.000, 20.000, yaw 1.571)`
once, and `tf2_echo map default_robot/base_link` reads (10.000, 20.930, yaw 1.571): 1 m along
`pointlio_odom`'s x minus the 0.07 m the pitched mount puts the base behind the lidar, swung onto
map's y by the quarter turn. `tf2_echo default_robot/odom default_robot/base_link` reads
(0.930, 0, 0) with all three RPY angles zero — the mount's 0.5 m of height and 0.25 rad of pitch
have been projected away.

## Project layout

```
.                           # = the ROS 2 package syncai_lio_bridge (build_type: ament_cargo)
├── package.xml
├── Cargo.toml / Cargo.lock
├── rustfmt.toml / clippy.toml / .editorconfig
├── launch/lio_bridge.launch.py     # reads robot_id from system.ini; identical to the C++ one
├── src/
│   ├── main.rs
│   └── lio_bridge_node/
│       ├── mod.rs          # wiring: parameters → publishers → one worker → subscriptions + timer
│       ├── parameters.rs   # the six load-time parameters, Hz -> period, seconds -> tolerance
│       ├── state.rs        # the worker payload, sample intake, and the publish tick
│       ├── subscribers.rs  # the four subscriptions and their QoS; geometry_msgs -> geometry
│       ├── geometry.rs     # Quaternion / Transform / Pose2D: the tf2 arithmetic  (pure, unit-tested)
│       ├── tf.rs           # the TF buffer and lookup that replace tf2_ros         (pure, unit-tested)
│       └── bridge.rs       # odom->base and map->odom, the two formulas above      (pure, unit-tested)
│
│   # Only for developing this repo on its own; SyncAI-Robot-Workspace does not use these
└── .devcontainer/          # Dockerfile + devcontainer.json
```

`geometry.rs`, `tf.rs` and `bridge.rs` are pure functions over plain Rust types — the ROS messages
are converted at the subscription boundary — so their tests need neither a ROS environment nor the
robot, the same split `tf.rs` has in syncai_robot_state.

## Differences from the C++ version

* **TF is handled by this package, not by tf2 / tf2_ros.** rclrs has no binding for either, so:
  * The listener is an own subscription to `/tf` and `/tf_static` feeding the buffer in
    `src/lio_bridge_node/tf.rs`. It keeps only the newest transform per child frame, with no time
    history, no interpolation and no cache expiry. The C++ looks up at `tf2::TimePointZero`, which
    resolves to the newest sample anyway, so nothing is lost on the lookups this node performs.
    Consequence, shared with the C++ version: if the localizer dies after relocalizing, its last
    `map → pointlio_odom` keeps being bridged in both versions.
  * The broadcaster is an own publisher on `/tf` with `tf2_ros::TransformBroadcaster`'s QoS
    (reliable, depth 100), sending one `TransformStamped` per message exactly as the C++ does.
  * The transform arithmetic (`tf2::Transform`, `tf2::Quaternion`, `tf2::Matrix3x3`) is
    reimplemented in `src/lio_bridge_node/geometry.rs`. The yaw is `atan2(R[1][0], R[0][0])`,
    the same formula the C++ `project_2d()` reads off the matrix — not `tf2::getYaw`.
* **A malformed `pointlio/lio_odom` pose is dropped, not cached.** A pose with a NaN or a non-unit
  quaternion keeps the previous pose and logs a throttled warning. The C++ would compose it and
  broadcast a NaN transform, which every tf2 consumer then rejects with a less useful message. The
  real Point-LIO never produces one; this guards against a third-party publisher on the topic.
* **Parameters are declared read-only**, so `ros2 param set` is rejected. The C++ reads each
  parameter once in its constructor and has no `add_on_set_parameters_callback`, so a set there
  appears to succeed and changes nothing. Same effective behaviour, made visible.
* **`publish_rate` and `transform_tolerance` are validated.** The C++ divides `1 / publish_rate`
  straight into a wall timer, so 0 or a negative rate is undefined there; here a non-positive or
  non-finite rate falls back to 1 Hz and the period is clamped to 1 ms..1 h. `rclcpp::Duration`
  accepts a negative `transform_tolerance` and back-dates the stamp; a Rust `Duration` cannot, so
  a negative, NaN or infinite tolerance is logged and treated as 0.
* **One worker instead of the default executor.** The four subscriptions and the timer run on a
  single rclrs `Worker`, whose callbacks never overlap — the same guarantee the C++ gets from its
  single-threaded executor, and the reason there is no mutex anywhere in this port.
* **There is no SIGINT handler.** rclrs does not handle signals, so Ctrl-C / `ros2 launch`
  shutdown terminates the process with the default action and `Drop` does not run. A process
  backgrounded with `&` from a non-interactive shell script ignores SIGINT; stop it with SIGTERM
  in scripts.
* **`lidar_frame` is `<robot_id>/lidar_top_imu`, not `<robot_id>/lidar_top`** (2026-10), in the
  launch file and the code default. The C++ version, and this port until then, mapped the
  Point-LIO body through `lidar_top` as if the body frame were the lidar; it is the lidar's
  built-in IMU, `t_il` (~5 cm on the MID360) away, which put `base_link` ~2.3 cm sideways in the
  plane. SyncAI-Robot-Workspace's URDF has carried both links since its PR #44 — `lidar_top`
  for the lidar, the frame its cloud is stamped with, and `lidar_top_imu` under it at `-t_il` —
  and this is the one place the launch file differs from the C++ one.
* **Log lines carry a `[LioBridge]` / `[Parameters]` prefix**, following the convention of the
  other Rust nodes. The message text after the prefix is the C++ text, so the diagnosis table
  above still greps.

## Gotchas (inherited from the C++ version)

- **`use_sim_time` defaults to `false`, and that default is load-bearing.** On the real robot
  there is no `/clock`; with `use_sim_time: true` `now()` returns zero and every TF stamp and odom
  header is garbage. The byobu session launches this node with no arguments, so do not flip the
  launch default without also editing `config/sessions/start_nav.yaml` in the Workspace.
- **`lidar_frame` must name a real frame.** It comes from the URDF's `lidar_top_joint` and
  `lidar_top_imu_joint`, published by `bringup` — so this node needs a Workspace whose URDF has
  the `lidar_top_imu` link (PR #44 onwards). Without it the node publishes *nothing at all* — not even the odom chain — because
  the extrinsic lookup gates the whole timer body.
- **The odom pose is a planar projection**, so `position.z` is always 0 and the orientation only
  has a yaw component. Do not use this topic as a 3D pose source; `pointlio/lio_odom` is the full
  6-DOF estimate.
- **Odometry covariance is never populated** — the pose and twist covariance arrays stay all-zero.
- **The twist is copied from the LIO body frame into a message whose `child_frame_id` is
  `base_link`,** without rotating by the lidar mount extrinsic. With the 0.25 rad mount pitch this
  is roughly a 3% underestimate of forward speed — fine for the consumers listed above, but not
  exact.
- **LIO drift shows up as a moving `map → odom`.** That is the correction doing its job; a
  *jumping* correction instead points at the localizer, not at this node.

## rclrs notes

* **The executor comes before the node.** `Context::default_from_env()?.create_basic_executor()`,
  then `executor.create_node(..)`, then `executor.spin(..)`.
* **A Worker is rclrs's callback group.** `node.create_worker(payload)` gives callbacks of the
  shape `FnMut(&mut Payload, Msg)`; callbacks under one worker run one at a time.
* **Timers default to steady time**, which is the C++ `create_wall_timer`: the publish cadence
  does not follow `/clock` under `use_sim_time`, while the stamps written into the messages come
  from the node clock and do.
* **Message types come from `ros-env`** (`use ros_env::nav_msgs::msg::Odometry;`). Message packages
  go in `<depend>` in `package.xml`, **not** in `Cargo.toml`; without the `<depend>`, colcon does
  not put the package on `AMENT_PREFIX_PATH` and `ros_env::<pkg>` does not exist.
* **`ros-env` versions**: this package uses `ros-env` 0.2 while rclrs uses 0.3 internally, and their
  message types are incompatible. Do not use rclrs helpers that return message types (e.g.
  `Time::to_ros_msg`); the stamp is built by hand from `Time::nsec` in `state.rs`.
* **`tf2_msgs` is the one message package not rebuilt by the underlay.** The official
  `ros-humble-tf2-msgs` deb already ships generated Rust bindings at `share/tf2_msgs/rust`, which
  is what this package links against. If a future base image stops shipping them, add geometry2's
  `tf2_msgs` to the underlay's `.repos`.
* **The underlay is built with `CARGO_RESOLVER_INCOMPATIBLE_RUST_VERSIONS=fallback`**, because the
  image pins Rust 1.85 (rclrs's own `rust-version`) while rclrs's unlocked dependencies drift past
  it. See the Dockerfile.

## References

* [ros2_rust](https://github.com/ros2-rust/ros2_rust)
* [rclrs examples](https://github.com/ros2-rust/examples/tree/main/rclrs)
* [rclrs on docs.rs](https://docs.rs/rclrs)
