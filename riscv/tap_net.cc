// See LICENSE for license details.
#include "tap_net.h"

#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <linux/if.h>
#include <linux/if_tun.h>

tap_net_t::tap_net_t(const std::string& name)
  : tap_name(name), tap_fd(-1), rx_callback(nullptr)
{
}

tap_net_t::~tap_net_t() {
  if (tap_fd >= 0) {
    close(tap_fd);
    tap_fd = -1;
  }
}

bool tap_net_t::init() {
  // Open TUN/TAP device
  tap_fd = open("/dev/net/tun", O_RDWR);
  if (tap_fd < 0) {
    fprintf(stderr, "tap_net: failed to open /dev/net/tun: %s\n", strerror(errno));
    fprintf(stderr, "tap_net: hint - try: sudo ip tuntap add dev %s mode tap user $USER\n", tap_name.c_str());
    return false;
  }

  // Configure as TAP device (layer 2, Ethernet frames)
  struct ifreq ifr;
  memset(&ifr, 0, sizeof(ifr));
  ifr.ifr_flags = IFF_TAP | IFF_NO_PI;  // TAP device, no packet info header
  strncpy(ifr.ifr_name, tap_name.c_str(), IFNAMSIZ - 1);

  if (ioctl(tap_fd, TUNSETIFF, &ifr) < 0) {
    fprintf(stderr, "tap_net: failed to configure TAP device '%s': %s\n",
            tap_name.c_str(), strerror(errno));
    fprintf(stderr, "tap_net: hint - create TAP device first:\n");
    fprintf(stderr, "  sudo ip tuntap add dev %s mode tap user $USER\n", tap_name.c_str());
    fprintf(stderr, "  sudo ip addr add 10.0.2.2/24 dev %s\n", tap_name.c_str());
    fprintf(stderr, "  sudo ip link set %s up\n", tap_name.c_str());
    close(tap_fd);
    tap_fd = -1;
    return false;
  }

  // Set non-blocking
  int flags = fcntl(tap_fd, F_GETFL, 0);
  fcntl(tap_fd, F_SETFL, flags | O_NONBLOCK);

  fprintf(stderr, "tap_net: TAP device '%s' opened successfully (fd=%d)\n",
          ifr.ifr_name, tap_fd);
  tap_name = ifr.ifr_name;  // Update with actual name

  return true;
}

void tap_net_t::send_packet(const uint8_t* data, size_t len) {
  if (tap_fd < 0 || len == 0)
    return;

  ssize_t n = write(tap_fd, data, len);
  if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
    fprintf(stderr, "tap_net: write error: %s\n", strerror(errno));
  }
}

void tap_net_t::poll(int timeout_ms) {
  if (tap_fd < 0 || !rx_callback)
    return;

  struct pollfd pfd;
  pfd.fd = tap_fd;
  pfd.events = POLLIN;
  pfd.revents = 0;

  int ret = ::poll(&pfd, 1, timeout_ms);
  if (ret > 0 && (pfd.revents & POLLIN)) {
    // Read available packets
    while (true) {
      ssize_t n = read(tap_fd, rx_buffer, sizeof(rx_buffer));
      if (n > 0) {
        rx_callback(rx_buffer, n);
      } else if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
          break;  // No more data
        fprintf(stderr, "tap_net: read error: %s\n", strerror(errno));
        break;
      } else {
        break;  // n == 0
      }
    }
  }
}
