#include "network_isolation.h"

// clang-format off
#include <ifaddrs.h>
#include <sched.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <net/route.h>
#include <sys/ioctl.h>
#include <sys/types.h>
// clang-format on

#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <type_traits>
#include <utility>

namespace ros2 {

[[noreturn]] inline void Throw(const char* condition, const char* func,
                               const char* file, int line) {
  std::ostringstream oss;
  oss << "Condition failed: (" << condition << ") in function " << func
      << " at " << file << ":" << line;
  throw std::runtime_error(oss.str());
}

class ScopeExit {
 public:
  ScopeExit(const ScopeExit&) = delete;
  ScopeExit& operator=(const ScopeExit&) = delete;
  ScopeExit(ScopeExit&&) = delete;
  ScopeExit& operator=(ScopeExit&&) = delete;

  explicit ScopeExit(std::function<void()> func)
      : func_(std::move(func)), active_(true) {}

  ~ScopeExit() {
    if (active_ && func_) {
      func_();
    }
  }

  // Disarm: cancels the scope exit
  void Disarm() { active_ = false; }

 private:
  std::function<void()> func_;
  bool active_;
};

#define THROW_UNLESS(condition)                                            \
  do {                                                                     \
    static_assert(std::is_convertible<decltype(condition), bool>::value,   \
                  "Condition must be bool-convertible.");                  \
    static_assert(!std::is_pointer<decltype(condition)>::value,            \
                  "For raw pointers, write 'ptr != nullptr' explicitly."); \
    if (!(condition)) {                                                    \
      Throw(#condition, __func__, __FILE__, __LINE__);                     \
    }                                                                      \
  } while (0)

#define CHECK_SYSCALL(result)                    \
  do {                                           \
    if (result != 0) {                           \
      const int error = errno;                   \
      std::ostringstream oss;                    \
      oss << "network_isolation.cc:" << __LINE__ \
          << ": error: " << strerror(error);     \
      throw std::runtime_error(oss.str());       \
    }                                            \
  } while (0)

void CreateLinuxNetworkNamespaces() {
  // If we've already been called once (e.g., by a parent process) there's no
  // need to do the work again (and trying to do so would fail). Our *.py file
  // repeats the same constant, so be sure to keep both copies in sync.
  constexpr char kFinishedMarker[] = "_ROS_CREATED_LINUX_NETWORK_NAMESPACES";
  if (getenv(kFinishedMarker) != nullptr) {
    return;
  }

  // Enter the namespaces.
  int result = unshare(CLONE_NEWUSER | CLONE_NEWNET | CLONE_NEWIPC);
  if (result != 0 && errno == EINVAL) {
    throw std::runtime_error(
        "CreateLinuxNetworkNamespaces failed to unshare(). One common cause "
        "of this is if any background threads have already been started.");
  }
  CHECK_SYSCALL(result);

  // Assert there is exactly one network interface.
  struct ifaddrs* ifaddr = nullptr;
  result = getifaddrs(&ifaddr);
  CHECK_SYSCALL(result);
  THROW_UNLESS(ifaddr != nullptr);
  ScopeExit ifaddr_cleanup([&]() {
    freeifaddrs(ifaddr);
  });
  THROW_UNLESS(ifaddr->ifa_next == nullptr);

  // Create a socket to do ioctl stuff on.
  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  THROW_UNLESS(fd >= 0);
  ScopeExit fd_cleanup([&]() {
    close(fd);
  });

  // Check what flags are set on the interface.
  struct ifreq ioctl_request = {};
  strncpy(ioctl_request.ifr_name, ifaddr->ifa_name, IFNAMSIZ);
  result = ioctl(fd, SIOCGIFFLAGS, &ioctl_request);
  CHECK_SYSCALL(result);

  // Expect a loopback interface.
  THROW_UNLESS(ioctl_request.ifr_flags & IFF_LOOPBACK);

  // Enable multicast and bring up interface.
  ioctl_request.ifr_flags |= IFF_MULTICAST;
  ioctl_request.ifr_flags |= IFF_UP;
  result = ioctl(fd, SIOCSIFFLAGS, &ioctl_request);
  CHECK_SYSCALL(result);

  // For programs that use both LCM and ROS, we need an LCM route ala
  //  sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo
  struct rtentry route = {};
  auto* dest = reinterpret_cast<struct sockaddr_in*>(&route.rt_dst);
  dest->sin_family = AF_INET;
  dest->sin_addr.s_addr = inet_addr("224.0.0.0");
  auto* mask = reinterpret_cast<struct sockaddr_in*>(&route.rt_genmask);
  mask->sin_family = AF_INET;
  mask->sin_addr.s_addr = inet_addr("240.0.0.0");
  std::string device{"lo"};
  route.rt_dev = device.data();
  route.rt_flags = RTF_UP;
  result = ioctl(fd, SIOCADDRT, &route);
  CHECK_SYSCALL(result);

  // Success!
  result = setenv(kFinishedMarker, "1", 1);
  CHECK_SYSCALL(result);
}

}  // namespace ros2
