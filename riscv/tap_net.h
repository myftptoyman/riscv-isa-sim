// See LICENSE for license details.
#ifndef _RISCV_TAP_NET_H
#define _RISCV_TAP_NET_H

#include <cstdint>
#include <string>
#include <functional>

// Callback type for receiving packets
using tap_rx_callback_t = std::function<void(const uint8_t*, size_t)>;

class tap_net_t {
public:
  tap_net_t(const std::string& tap_name = "tap0");
  ~tap_net_t();

  // Initialize TAP device
  bool init();

  // Send a packet from guest to network
  void send_packet(const uint8_t* data, size_t len);

  // Poll for network activity (call periodically)
  void poll(int timeout_ms = 0);

  // Set callback for received packets (from network to guest)
  void set_rx_callback(tap_rx_callback_t callback) {
    rx_callback = callback;
  }

  // Check if initialized
  bool is_initialized() const { return tap_fd >= 0; }

  // Get TAP device name
  const std::string& get_name() const { return tap_name; }

private:
  std::string tap_name;
  int tap_fd;
  tap_rx_callback_t rx_callback;

  // Read buffer
  uint8_t rx_buffer[65536];
};

#endif // _RISCV_TAP_NET_H
