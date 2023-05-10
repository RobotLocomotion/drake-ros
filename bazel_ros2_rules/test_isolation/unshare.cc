
#include <cassert>
#include <cstring>
#include <iostream>
 
#include <errno.h>
#include <sched.h>
#include <sys/ioctl.h>
#include <net/if.h>

// #define _GNU_SOURCE     /* To get defns of NI_MAXSERV and NI_MAXHOST */
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>
#include <ifaddrs.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/if_link.h>


void die(const char * message) {
    std::cout << "TEST_ISOLATION: " << message << ".\n";
    exit(-1);
}


int main(int argc, char ** argv) {
    // Create linux namespaces for the current process
    // * A new user namespace to avoid needing CAP_SYS_ADMIN to create
    //   network and IPC namespaces
    // * A new network namespace to prevent cross-talk via the network
    // * A new IPC namespaces to prevent cross-talk via shared memrory
    int result = unshare(CLONE_NEWUSER | CLONE_NEWNET | CLONE_NEWIPC);

    if (result != 0) {
        perror("unshare");
        return result;
    }


    // Assert there is exactly one network interface
    struct ifaddrs *ifaddr;

    if (-1 == getifaddrs(&ifaddr)) {
        perror("getifaddrs");
        die("could not get network interfaces");
    }
    if (nullptr == ifaddr) {
        die("there are no network interfaces");
    }
    if (nullptr != ifaddr->ifa_next) {
        die("there are multiple network interfaces");
    }

    // Need a socket to do ioctl stuff on
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if( fd < 0 ){
        die("could not open a socket");
    }

    struct ifreq ioctl_request;

    // Check what flags are set on the interface
    strncpy(ioctl_request.ifr_name, ifaddr->ifa_name, IFNAMSIZ);
    int err = ioctl(fd, SIOCGIFFLAGS, &ioctl_request);
    if (0 != err) {
        perror("ioctl");
        die("failed to get interface flags");
    }

    // Expecting a loopback interface.
    if (!(ioctl_request.ifr_flags & IFF_LOOPBACK)) {
        die("the only interface is not a loopback interface");
    }

    // Enable multicast if it's not already enabled.
    if (!(ioctl_request.ifr_flags & IFF_MULTICAST)) {
        std::cout << "Multicast is not enabled on the interface\n";

        ioctl_request.ifr_flags |= IFF_MULTICAST;

        int err = ioctl(fd, SIOCSIFFLAGS, &ioctl_request);
        if (0 != err) {
            perror("ioctl");
            die("failed to set interface flags");
        }
    }

    freeifaddrs(ifaddr);

    // TODO(sloretz) Exec test process


    // Unhappy code so I see output from bazel_test without having to look up the bazel CLI flag
    return -1;
}
