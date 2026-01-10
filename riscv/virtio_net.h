// See LICENSE for license details.
#ifndef _RISCV_VIRTIO_NET_H
#define _RISCV_VIRTIO_NET_H

#include "config.h"
#include "virtio.h"
#include <string>
#include <queue>
#include <vector>

#ifdef HAVE_SLIRP
#include "slirp_net.h"
#endif

// VirtIO Network device ID (standard device type)
#define VIRTIO_ID_NET           0x01

// VirtIO Network feature bits (VirtIO spec section 5.1.3)
#define VIRTIO_NET_F_CSUM              (1ULL << 0)   // Host handles pkts w/ partial csum
#define VIRTIO_NET_F_GUEST_CSUM        (1ULL << 1)   // Guest handles pkts w/ partial csum
#define VIRTIO_NET_F_MAC               (1ULL << 5)   // Device has given MAC address
#define VIRTIO_NET_F_GSO               (1ULL << 6)   // Deprecated
#define VIRTIO_NET_F_GUEST_TSO4        (1ULL << 7)   // Guest can receive TSOv4
#define VIRTIO_NET_F_GUEST_TSO6        (1ULL << 8)   // Guest can receive TSOv6
#define VIRTIO_NET_F_GUEST_ECN         (1ULL << 9)   // Guest can receive TSO with ECN
#define VIRTIO_NET_F_GUEST_UFO         (1ULL << 10)  // Guest can receive UFO
#define VIRTIO_NET_F_HOST_TSO4         (1ULL << 11)  // Host can receive TSOv4
#define VIRTIO_NET_F_HOST_TSO6         (1ULL << 12)  // Host can receive TSOv6
#define VIRTIO_NET_F_HOST_ECN          (1ULL << 13)  // Host can receive TSO with ECN
#define VIRTIO_NET_F_HOST_UFO          (1ULL << 14)  // Host can receive UFO
#define VIRTIO_NET_F_MRG_RXBUF         (1ULL << 15)  // Driver can merge receive buffers
#define VIRTIO_NET_F_STATUS            (1ULL << 16)  // Configuration status field available
#define VIRTIO_NET_F_CTRL_VQ           (1ULL << 17)  // Control channel available
#define VIRTIO_NET_F_CTRL_RX           (1ULL << 18)  // Control channel RX mode support
#define VIRTIO_NET_F_CTRL_VLAN         (1ULL << 19)  // Control channel VLAN filtering
#define VIRTIO_NET_F_GUEST_ANNOUNCE    (1ULL << 21)  // Guest can send gratuitous pkts
#define VIRTIO_NET_F_MQ                (1ULL << 22)  // Device supports multiqueue
#define VIRTIO_NET_F_CTRL_MAC_ADDR     (1ULL << 23)  // Set MAC address through control channel
#define VIRTIO_NET_F_MTU               (1ULL << 3)   // Initial MTU advice

// Network status bits
#define VIRTIO_NET_S_LINK_UP           1
#define VIRTIO_NET_S_ANNOUNCE          2

// VirtIO network header flags
#define VIRTIO_NET_HDR_F_NEEDS_CSUM    1
#define VIRTIO_NET_HDR_F_DATA_VALID    2
#define VIRTIO_NET_HDR_F_RSC_INFO      4

// VirtIO network header GSO types
#define VIRTIO_NET_HDR_GSO_NONE        0
#define VIRTIO_NET_HDR_GSO_TCPV4       1
#define VIRTIO_NET_HDR_GSO_UDP         3
#define VIRTIO_NET_HDR_GSO_TCPV6       4
#define VIRTIO_NET_HDR_GSO_ECN         0x80

// VirtIO network packet header (VirtIO spec section 5.1.6)
// This header is prepended to each packet
struct virtio_net_hdr {
  uint8_t flags;
  uint8_t gso_type;
  uint16_t hdr_len;
  uint16_t gso_size;
  uint16_t csum_start;
  uint16_t csum_offset;
  // uint16_t num_buffers;  // Only present if VIRTIO_NET_F_MRG_RXBUF negotiated
};

#define VIRTIO_NET_HDR_SIZE_BASE 10  // Size without num_buffers
#define VIRTIO_NET_HDR_SIZE_MRG  12  // Size with num_buffers (when MRG_RXBUF negotiated)

// VirtIO network configuration space (VirtIO spec section 5.1.4)
struct virtio_net_config {
  uint8_t mac[6];              // MAC address
  uint16_t status;             // Link status (if VIRTIO_NET_F_STATUS)
  uint16_t max_virtqueue_pairs;// Max multiqueue pairs (if VIRTIO_NET_F_MQ)
  uint16_t mtu;                // MTU (if VIRTIO_NET_F_MTU)
};

// Default queue size
#define VIRTIO_NET_QUEUE_SIZE  256

// Default MTU
#define VIRTIO_NET_DEFAULT_MTU 1500

// SLIRP configuration for integrated networking
struct virtio_net_slirp_config_t {
  bool enabled;
  std::vector<std::pair<uint16_t, uint16_t>> port_forwards; // host_port -> guest_port
  bool debug;

  virtio_net_slirp_config_t() :
    enabled(false), debug(false) {}
};

class virtio_net_t : public virtio_base_t {
public:
  // Queue indices
  static const uint32_t QUEUE_RX = 0;  // Receive (network to guest)
  static const uint32_t QUEUE_TX = 1;  // Transmit (guest to network)
  static const uint32_t NUM_QUEUES = 2;

#ifdef HAVE_SLIRP
  // Constructor for SLIRP mode
  virtio_net_t(simif_t* sim,
               abstract_interrupt_controller_t* intctrl,
               uint32_t interrupt_id,
               const virtio_net_slirp_config_t& slirp_config);
#endif

  virtual ~virtio_net_t();

  void tick(reg_t rtc_ticks) override;

protected:
  // virtio_base_t overrides
  uint32_t get_device_id() const override { return VIRTIO_ID_NET; }
  uint64_t get_device_features() const override;
  void handle_driver_features(uint64_t features) override;
  void handle_queue_notify(uint32_t queue_idx) override;
  bool read_config(reg_t offset, size_t len, uint8_t* bytes) override;
  bool write_config(reg_t offset, size_t len, const uint8_t* bytes) override;
  uint32_t get_config_size() const override { return sizeof(virtio_net_config); }
  void device_reset() override;

private:
  // Queue processing
  void process_tx_queue();
  void process_rx_queue();

  // Get header size based on negotiated features
  size_t get_hdr_size() const {
    return (negotiated_features & VIRTIO_NET_F_MRG_RXBUF) ?
           VIRTIO_NET_HDR_SIZE_MRG : VIRTIO_NET_HDR_SIZE_BASE;
  }

  // Configuration
  virtio_net_config config;
  uint64_t negotiated_features;

  // Tick counter for polling
  uint64_t tick_count;
  static const uint64_t POLL_INTERVAL = 100;  // Poll every N ticks

  // RX packet queue (received from network, waiting to be sent to guest)
  struct rx_packet {
    std::vector<uint8_t> data;
  };
  std::queue<rx_packet> rx_queue;

#ifdef HAVE_SLIRP
  // SLIRP networking
  slirp_net_t* slirp_net;
  void slirp_rx_callback(const uint8_t* data, size_t len);
  void init_slirp(const virtio_net_slirp_config_t& cfg);
#endif
};

// Factory functions for device registration
virtio_net_t* virtio_net_parse_from_fdt(const void* fdt, const sim_t* sim,
                                         reg_t* base, const std::vector<std::string>& sargs);
std::string virtio_net_generate_dts(const sim_t* sim, const std::vector<std::string>& sargs);

// Global SLIRP configuration
void set_virtio_net_slirp_config(const virtio_net_slirp_config_t& config);
const virtio_net_slirp_config_t& get_virtio_net_slirp_config();
bool is_virtio_net_slirp_enabled();

#endif // _RISCV_VIRTIO_NET_H
