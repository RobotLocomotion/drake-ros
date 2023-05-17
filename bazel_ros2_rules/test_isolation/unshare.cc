
#include <cassert>
#include <cstring>
#include <iostream>
#include <vector>
 
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
    std::cerr << "TEST_ISOLATION: " << message << ".\n";
    exit(-1);
}


int main(int argc, char ** argv) {
    if (argc < 2) {
        die("shim must be given a command to execute");
    }
    // TODO die if the first argument is not an absolute path, or doesn't exist.

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

    // Enable multicast
    ioctl_request.ifr_flags |= IFF_MULTICAST;
    // Bring up interface
    ioctl_request.ifr_flags |= IFF_UP;

    err = ioctl(fd, SIOCSIFFLAGS, &ioctl_request);
    if (0 != err) {
        perror("ioctl");
        die("failed to set interface flags");
    }

    freeifaddrs(ifaddr);

    // Have to copy new a new array that terminates with a null pointer.
    std::vector<char *> new_argv;
    for (int i = 1; i < argc; ++i) {
        new_argv.push_back(argv[i]);
    }
    new_argv.push_back(nullptr);

    // Exec a new process - should never return!
    execv(new_argv.at(0), &new_argv.at(0));

    perror("execv");
    die("Call to execv failed");
    return -1;
}
