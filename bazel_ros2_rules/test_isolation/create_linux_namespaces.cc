#include "create_linux_namespaces.h"

#include <errno.h>
#include <sched.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <sys/types.h>
#include <ifaddrs.h>
#include <string.h>

#include <iostream>

namespace bazel_ros2_rules {

void error(const char * message)
{
    std::cerr << "create_linux_namesapces: "
        << message << ":" << strerror(errno) << "\n";
}

bool create_linux_namespaces()
{
    int result = unshare(CLONE_NEWUSER | CLONE_NEWNET | CLONE_NEWIPC);

    if (result != 0) {
        error("failed to call unshare");
        return false;
    }

    // Assert there is exactly one network interface
    struct ifaddrs *ifaddr;

    if (-1 == getifaddrs(&ifaddr)) {
        error("could not get network interfaces");
        return false;
    }
    if (nullptr == ifaddr) {
        error("there are no network interfaces");
        return false;
    }
    if (nullptr != ifaddr->ifa_next) {
        error("there are multiple network interfaces");
        return false;
    }

    // Need a socket to do ioctl stuff on
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if( fd < 0 ){
        error("could not open a socket");
        freeifaddrs(ifaddr);
        return false;
    }

    struct ifreq ioctl_request;

    // Check what flags are set on the interface
    strncpy(ioctl_request.ifr_name, ifaddr->ifa_name, IFNAMSIZ);
    int err = ioctl(fd, SIOCGIFFLAGS, &ioctl_request);
    if (0 != err) {
        freeifaddrs(ifaddr);
        error("failed to get interface flags");
        return false;
    }

    // Expecting a loopback interface.
    if (!(ioctl_request.ifr_flags & IFF_LOOPBACK)) {
        error("the only interface is not a loopback interface");
        freeifaddrs(ifaddr);
        return false;
    }

    // Enable multicast
    ioctl_request.ifr_flags |= IFF_MULTICAST;
    // Bring up interface
    ioctl_request.ifr_flags |= IFF_UP;

    err = ioctl(fd, SIOCSIFFLAGS, &ioctl_request);
    if (0 != err) {
        error("failed to set interface flags");
        freeifaddrs(ifaddr);
        return false;
    }

    freeifaddrs(ifaddr);
    return true;
}
}  // namespace bazel_ros2_rules
