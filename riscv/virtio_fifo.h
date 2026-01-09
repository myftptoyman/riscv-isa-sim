// See LICENSE for license details.
#ifndef _RISCV_VIRTIO_FIFO_H
#define _RISCV_VIRTIO_FIFO_H

#include "virtio.h"
#include <string>
#include <queue>
#include <vector>

// VirtIO FIFO device ID (custom device type)
// Using a value in the vendor-specific range
#define VIRTIO_ID_FIFO          0x1F

// FIFO device configuration space
struct virtio_fifo_config {
  uint32_t max_tx_size;   // Maximum TX buffer size
  uint32_t max_rx_size;   // Maximum RX buffer size
  uint32_t flags;         // Device flags
};

// Default queue size
#define VIRTIO_FIFO_QUEUE_SIZE  256

// Default buffer sizes
#define VIRTIO_FIFO_MAX_TX_SIZE 4096
#define VIRTIO_FIFO_MAX_RX_SIZE 4096

class virtio_fifo_t : public virtio_base_t {
public:
  // Queue indices
  static const uint32_t QUEUE_TX = 0;  // Guest to Host (transmit)
  static const uint32_t QUEUE_RX = 1;  // Host to Guest (receive)
  static const uint32_t NUM_QUEUES = 2;

  virtio_fifo_t(simif_t* sim,
                abstract_interrupt_controller_t* intctrl,
                uint32_t interrupt_id,
                const std::string& socket_path);
  virtual ~virtio_fifo_t();

  void tick(reg_t rtc_ticks) override;

protected:
  // virtio_base_t overrides
  uint32_t get_device_id() const override { return VIRTIO_ID_FIFO; }
  uint64_t get_device_features() const override;
  void handle_driver_features(uint64_t features) override;
  void handle_queue_notify(uint32_t queue_idx) override;
  bool read_config(reg_t offset, size_t len, uint8_t* bytes) override;
  bool write_config(reg_t offset, size_t len, const uint8_t* bytes) override;
  uint32_t get_config_size() const override { return sizeof(virtio_fifo_config); }
  void device_reset() override;

private:
  // Socket handling
  bool init_socket();
  void close_socket();
  bool accept_connection();
  bool has_rx_data();
  ssize_t receive_data(void* buf, size_t max_len);
  ssize_t send_data(const void* buf, size_t len);

  // Queue processing
  void process_tx_queue();
  void process_rx_queue();

  // Configuration
  virtio_fifo_config config;
  std::string socket_path;

  // Socket state
  int listen_fd;
  int conn_fd;
  bool connected;

  // RX buffer for incoming data from host
  std::queue<uint8_t> rx_buffer;

  // Tick counter for polling
  uint64_t tick_count;
  static const uint64_t POLL_INTERVAL = 100;  // Poll every N ticks
};

// Factory functions for device registration
virtio_fifo_t* virtio_fifo_parse_from_fdt(const void* fdt, const sim_t* sim,
                                           reg_t* base, const std::vector<std::string>& sargs);
std::string virtio_fifo_generate_dts(const sim_t* sim, const std::vector<std::string>& sargs);

// Global socket path configuration
void set_virtio_fifo_socket_path(const std::string& path);
const std::string& get_virtio_fifo_socket_path();

#endif // _RISCV_VIRTIO_FIFO_H
