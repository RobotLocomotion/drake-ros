# Drake ROS Systems

This is a ROS 2 prototype of a solution to [robotlocomotion/Drake#9500](https://github.com/RobotLocomotion/drake/issues/9500).
It is similar to this ROS 1 prototype [`gizatt/drake_ros_systems`](https://github.com/gizatt/drake_ros_systems).
It explores ROS 2 equivalents of `LcmInterfaceSystem`, `LcmPublisherSystem`, and `LcmSubscriberSystem`.

# Code examples

Create a system that publishes `std_msgs::msg::String`.

```C++
#include <drake_ros_systems/ros_interface_system.h>
#include <drake_ros_systems/ros_publisher_system.h>

using drake_ros_systems::RosInterfaceSystem;
using drake_ros_systems::RosPublisherSystem;

// This system initializes ROS and calls spin()
// It creates a single ROS node called "drake_ros".
auto spin_system = RosInterfaceSystem::Make(/* optionally rclcpp init options? */);

const std::string topic{"chatter"};
const double period_sec{0.1};  // 10Hz

auto pub_system = RosPublisherSystem::Make<std_msgs::msg::String>(
    spin_system.get_ros_interface(), topic, period_sec);

auto pub_context = pub_system->CreateDefaultContext();
std_msgs::msg::String pub_msg;
pub_msg.data = "Hello from Drake";
pub_context->FixInputPort(0,  AbstractValue::Make(pub_msg));
pub_system->Publish(*pub_context);
```

Create a system that subscribes to `std_msgs::msg::String`.

```C++
#include <drake_ros_systems/ros_interface_system.h>
#include <drake_ros_systems/ros_subsciber_system.h>

using drake_ros_systems::RosInterfaceSystem;
using drake_ros_systems::RosSubscriberSystem;

// This system initializes ROS and calls spin()
// It creates a single ROS node called "drake_ros".
auto spin_system = RosInterfaceSystem(/* optionally rclcpp init options? */);

const std::string topic{"chatter"};

auto pub_system = RosSubscriberSystem::Make<std_msgs::msg::String>(
    spin_system.get_ros_interface(), topic, period_sec);

auto sub_context = sub_system->CreateDefaultContext();
// somehow this sub context is added to a diagram builder with the system
// so the subscriber can update that message

// huh...?
std_msgs::msg::String sub_msg = sub_context->get_output_stuff(...);
```

Could use an example of drake systems built with a diagram builder and connected to input/output ports of other systems.

