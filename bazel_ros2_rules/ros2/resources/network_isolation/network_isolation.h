#pragma once

namespace ros2 {

/// Creates linux namespaces suitable for isolating network traffic (for both
/// ROS 2 and LCM), and configures the network namespace to enable that traffic.
///
/// This is most typically used to isolate a test program from any other network
/// traffic on the same machine.
///
/// The new namespaces are:
/// - A new user namespace to avoid needing CAP_SYS_ADMIN to create network and
///   IPC namespaces.
/// - A new network namespace to prevent cross-talk via the network.
/// - A new IPC namespaces to prevent cross-talk via shared memory.
///
/// Upon return, the current process will be in the created namespaces. Any
/// future child processes that are subsequently launched will be in the same
/// namespace, so they will be able to talk amongst themselves and this process.
///
/// This function relies on updating environment variables to help do its job.
/// If code launches new subprocesses, be sure that the environment variables
/// are preserved when doing so. (This usually happens correctly by default;
/// you'd need to go out of your way to turn it off.)
///
/// @throws std::exception if this process has already launched any threads.
void CreateLinuxNetworkNamespaces();

}  // namespace ros2
