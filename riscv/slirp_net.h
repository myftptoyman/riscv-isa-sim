// See LICENSE for license details.
#ifndef _RISCV_SLIRP_NET_H
#define _RISCV_SLIRP_NET_H

#include "config.h"

#ifdef HAVE_SLIRP

#include <slirp/libslirp.h>
#include <string>
#include <queue>
#include <vector>
#include <functional>

// SLIRP network configuration
struct slirp_config_t {
  std::string net_addr;      // Network address (default: 10.0.2.0)
  std::string net_mask;      // Network mask (default: 255.255.255.0)
  std::string host_addr;     // Host/gateway address (default: 10.0.2.2)
  std::string guest_addr;    // Guest address (default: 10.0.2.15)
  std::string dns_addr;      // DNS address (default: 10.0.2.3)
  uint16_t host_port;        // Host port for forwarding (default: 8080)
  uint16_t guest_port;       // Guest port to forward to (default: 80)
  bool debug;                // Enable debug output

  slirp_config_t() :
    net_addr("10.0.2.0"),
    net_mask("255.255.255.0"),
    host_addr("10.0.2.2"),
    guest_addr("10.0.2.15"),
    dns_addr("10.0.2.3"),
    host_port(8080),
    guest_port(80),
    debug(false) {}
};

// Callback type for receiving packets
using slirp_rx_callback_t = std::function<void(const uint8_t*, size_t)>;

class slirp_net_t {
public:
  slirp_net_t(const slirp_config_t& config);
  ~slirp_net_t();

  // Initialize SLIRP
  bool init();

  // Send a packet from guest to network
  void send_packet(const uint8_t* data, size_t len);

  // Poll for network activity (call periodically)
  void poll(int timeout_ms = 0);

  // Set callback for received packets (from network to guest)
  void set_rx_callback(slirp_rx_callback_t callback) {
    rx_callback = callback;
  }

  // Check if initialized
  bool is_initialized() const { return slirp != nullptr; }

private:
  // SLIRP callbacks (static to match C API)
  static ssize_t send_packet_cb(const void* buf, size_t len, void* opaque);
  static void guest_error_cb(const char* msg, void* opaque);
  static int64_t clock_get_ns_cb(void* opaque);
  static void* timer_new_cb(SlirpTimerCb cb, void* cb_opaque, void* opaque);
  static void timer_free_cb(void* timer, void* opaque);
  static void timer_mod_cb(void* timer, int64_t expire_time, void* opaque);
  static void register_poll_fd_cb(int fd, void* opaque);
  static void unregister_poll_fd_cb(int fd, void* opaque);
  static void notify_cb(void* opaque);
  static int add_poll_cb(int fd, int events, void* opaque);
  static int get_revents_cb(int idx, void* opaque);

  slirp_config_t config;
  Slirp* slirp;
  slirp_rx_callback_t rx_callback;

  // Poll state
  struct pollfd* poll_fds;
  int poll_fd_count;
  int poll_fd_capacity;

  // Timer support
  struct timer_entry {
    SlirpTimerCb cb;
    void* cb_opaque;
    int64_t expire_time;
    bool active;
  };
  std::vector<timer_entry> timers;
};

#endif // HAVE_SLIRP

#endif // _RISCV_SLIRP_NET_H
