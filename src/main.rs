mod lio_bridge_node;

use rclrs::*;

use lio_bridge_node::LioBridgeNode;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // rclrs inverts the rclcpp order: the executor comes first, and the node is created from it.
    let mut executor = Context::default_from_env()?.create_basic_executor();
    // The node name the C++ version uses, so this stays a drop-in replacement: the launch file
    // names it, and `ros2 node list` must not change.
    let node = executor.create_node("lio_bridge_node")?;
    let _lio_bridge = LioBridgeNode::new(node)?;

    // Single-threaded is enough, as in the C++ main: the subscription callbacks only latch the
    // newest sample and the timer does all the work, so there is nothing to overlap.
    //
    // No SIGINT handler: rclrs does not install one, so Ctrl-C ends the process without running
    // Drop. In scripts, stop a backgrounded node with SIGTERM.
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
