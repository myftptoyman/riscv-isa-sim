// See LICENSE for license details.
#include "virtio_net.h"
#include "dts.h"
#include "libfdt.h"
#include "platform.h"
#include "sim.h"

#include <cstring>
#include <sstream>

// Global SLIRP configuration
static virtio_net_slirp_config_t g_virtio_net_slirp_config;

void set_virtio_net_slirp_config(const virtio_net_slirp_config_t &config) {
  fprintf(stderr,
          "set_virtio_net_slirp_config: enabled=%d, port_forwards=%zu\n",
          config.enabled, config.port_forwards.size());
  g_virtio_net_slirp_config = config;
}

const virtio_net_slirp_config_t &get_virtio_net_slirp_config() {
  return g_virtio_net_slirp_config;
}

bool is_virtio_net_slirp_enabled() { return g_virtio_net_slirp_config.enabled; }

// Global TAP configuration
static std::string g_virtio_net_tap_name;

void set_virtio_net_tap_name(const std::string &name) {
  fprintf(stderr, "set_virtio_net_tap_name: name=%s\n", name.c_str());
  g_virtio_net_tap_name = name;
}

const std::string &get_virtio_net_tap_name() { return g_virtio_net_tap_name; }

bool is_virtio_net_tap_enabled() { return !g_virtio_net_tap_name.empty(); }

// TAP mode constructor
virtio_net_t::virtio_net_t(simif_t *sim,
                           abstract_interrupt_controller_t *intctrl,
                           uint32_t interrupt_id, const std::string &tap_name)
    : virtio_base_t(sim, intctrl, interrupt_id, NUM_QUEUES,
                    VIRTIO_NET_QUEUE_SIZE),
      negotiated_features(0), tick_count(0), burst_poll_remaining(0),
      tap_net(nullptr)
#ifdef HAVE_SLIRP
      ,
      slirp_net(nullptr)
#endif
{
  // Initialize configuration with a fixed MAC address
  config.mac[0] = 0x52;
  config.mac[1] = 0x54;
  config.mac[2] = 0x00;
  config.mac[3] = 0x12;
  config.mac[4] = 0x34;
  config.mac[5] = 0x56;
  config.status = VIRTIO_NET_S_LINK_UP;
  config.max_virtqueue_pairs = 1;
  config.mtu = VIRTIO_NET_DEFAULT_MTU;

  // Initialize TAP
  init_tap(tap_name);
}

void virtio_net_t::init_tap(const std::string &tap_name) {
  tap_net = new tap_net_t(tap_name);
  if (!tap_net->init()) {
    delete tap_net;
    tap_net = nullptr;
    fprintf(stderr, "virtio-net: failed to initialize TAP device '%s'\n",
            tap_name.c_str());
    return;
  }

  // Set up receive callback
  tap_net->set_rx_callback([this](const uint8_t *data, size_t len) {
    this->tap_rx_callback(data, len);
  });

  fprintf(stderr, "virtio-net: TAP networking enabled (device: %s)\n",
          tap_net->get_name().c_str());
  fprintf(stderr, "virtio-net: MAC address %02x:%02x:%02x:%02x:%02x:%02x\n",
          config.mac[0], config.mac[1], config.mac[2], config.mac[3],
          config.mac[4], config.mac[5]);
}

void virtio_net_t::tap_rx_callback(const uint8_t *data, size_t len) {
  static uint64_t rx_total = 0;
  rx_total++;

  fprintf(stderr, "virtio-net: TAP RX packet #%lu len=%zu\n", rx_total, len);

  // Drop packet if RX queue is full
  if (rx_queue.size() >= MAX_RX_QUEUE_SIZE) {
    fprintf(stderr, "virtio-net: TAP RX DROP! queue full (%zu)\n",
            rx_queue.size());
    return;
  }

  // Create a packet with VirtIO net header prepended
  size_t hdr_size = get_hdr_size();
  rx_packet pkt;
  pkt.data.resize(hdr_size + len);

  // Fill in VirtIO net header
  memset(pkt.data.data(), 0, hdr_size);
  virtio_net_hdr *hdr = reinterpret_cast<virtio_net_hdr *>(pkt.data.data());
  hdr->flags = 0;
  hdr->gso_type = VIRTIO_NET_HDR_GSO_NONE;

  if (negotiated_features & VIRTIO_NET_F_MRG_RXBUF) {
    uint16_t num_buffers = 1;
    pkt.data[10] = num_buffers & 0xff;
    pkt.data[11] = (num_buffers >> 8) & 0xff;
  }

  // Copy Ethernet frame after header
  memcpy(pkt.data.data() + hdr_size, data, len);

  // Queue packet for delivery to guest
  rx_queue.push(std::move(pkt));
}

#ifdef HAVE_SLIRP

virtio_net_t::virtio_net_t(simif_t *sim,
                           abstract_interrupt_controller_t *intctrl,
                           uint32_t interrupt_id,
                           const virtio_net_slirp_config_t &slirp_config)
    : virtio_base_t(sim, intctrl, interrupt_id, NUM_QUEUES,
                    VIRTIO_NET_QUEUE_SIZE),
      negotiated_features(0), tick_count(0), burst_poll_remaining(0),
      tap_net(nullptr), slirp_net(nullptr) {
  // Initialize configuration with a fixed MAC address
  // Using locally administered address: 52:54:00:12:34:56
  config.mac[0] = 0x52;
  config.mac[1] = 0x54;
  config.mac[2] = 0x00;
  config.mac[3] = 0x12;
  config.mac[4] = 0x34;
  config.mac[5] = 0x56;
  config.status = VIRTIO_NET_S_LINK_UP;
  config.max_virtqueue_pairs = 1;
  config.mtu = VIRTIO_NET_DEFAULT_MTU;

  // Initialize SLIRP
  init_slirp(slirp_config);
}

void virtio_net_t::init_slirp(const virtio_net_slirp_config_t &cfg) {
  slirp_config_t slirp_cfg;
  slirp_cfg.debug = cfg.debug;

  // Set up port forwarding - use first pair if available
  if (!cfg.port_forwards.empty()) {
    slirp_cfg.host_port = cfg.port_forwards[0].first;
    slirp_cfg.guest_port = cfg.port_forwards[0].second;
  }

  slirp_net = new slirp_net_t(slirp_cfg);
  if (!slirp_net->init()) {
    delete slirp_net;
    slirp_net = nullptr;
    fprintf(stderr, "virtio-net: failed to initialize SLIRP\n");
    return;
  }

  // Set up receive callback
  slirp_net->set_rx_callback([this](const uint8_t *data, size_t len) {
    this->slirp_rx_callback(data, len);
  });

  // Set up QEMU-style can_receive callback for backpressure
  slirp_net->set_can_receive_callback(
      [this]() -> bool { return this->rx_queue.size() < MAX_RX_QUEUE_SIZE; });

  fprintf(
      stderr,
      "virtio-net: SLIRP networking enabled (standard VirtIO-Net for Linux)\n");
  fprintf(stderr, "virtio-net: MAC address %02x:%02x:%02x:%02x:%02x:%02x\n",
          config.mac[0], config.mac[1], config.mac[2], config.mac[3],
          config.mac[4], config.mac[5]);

  // Print port forwarding info
  for (const auto &pf : cfg.port_forwards) {
    fprintf(stderr, "virtio-net: port forward host:%d -> guest:%d\n", pf.first,
            pf.second);
  }
}

void virtio_net_t::slirp_rx_callback(const uint8_t *data, size_t len) {
  static bool debug_virtio = getenv("DEBUG_VIRTIO") != nullptr;
  static uint64_t rx_total = 0;
  static uint64_t rx_dropped = 0;
  static uint64_t last_drop_msg = 0;
  rx_total++;

  // Drop packet if RX queue is full (flow control)
  if (rx_queue.size() >= MAX_RX_QUEUE_SIZE) {
    rx_dropped++;
    if (rx_total - last_drop_msg >= 100 || rx_dropped == 1) {
      fprintf(stderr,
              "virtio-net: RX DROP! queue full (%zu), total=%lu, dropped=%lu\n",
              rx_queue.size(), rx_total, rx_dropped);
      last_drop_msg = rx_total;
    }
    return;
  }

  if (debug_virtio) {
    fprintf(stderr,
            "virtio-net: slirp_rx_callback len=%zu (packet from network to "
            "guest)\n",
            len);
    // Dump first 20 bytes of Ethernet frame (dst MAC, src MAC, ethertype)
    if (len >= 14) {
      fprintf(stderr, "  dst MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", data[0],
              data[1], data[2], data[3], data[4], data[5]);
      fprintf(stderr, "  src MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", data[6],
              data[7], data[8], data[9], data[10], data[11]);
      fprintf(stderr, "  ethertype: %02x%02x\n", data[12], data[13]);
    }
  }

  // Create a packet with VirtIO net header prepended

  // Use the correct header size based on negotiated features
  size_t hdr_size = get_hdr_size();
  rx_packet pkt;
  pkt.data.resize(hdr_size + len);

  // Fill in VirtIO net header
  memset(pkt.data.data(), 0, hdr_size);
  virtio_net_hdr *hdr = reinterpret_cast<virtio_net_hdr *>(pkt.data.data());
  hdr->flags = 0;
  hdr->gso_type = VIRTIO_NET_HDR_GSO_NONE;

  // If MRG_RXBUF is negotiated, set num_buffers to 1 (single buffer for this
  // packet)
  if (negotiated_features & VIRTIO_NET_F_MRG_RXBUF) {
    // num_buffers is at offset 10 (after the 10-byte base header),
    // little-endian
    uint16_t num_buffers = 1;
    pkt.data[10] = num_buffers & 0xff;
    pkt.data[11] = (num_buffers >> 8) & 0xff;
  }

  // Copy Ethernet frame after header
  memcpy(pkt.data.data() + hdr_size, data, len);

  // Queue packet for delivery to guest
  rx_queue.push(std::move(pkt));
  if (debug_virtio) {
    fprintf(stderr, "virtio-net: rx_queue.size()=%zu after push\n",
            rx_queue.size());
  }
}

#endif // HAVE_SLIRP

virtio_net_t::~virtio_net_t() {
  if (tap_net) {
    delete tap_net;
    tap_net = nullptr;
  }
#ifdef HAVE_SLIRP
  if (slirp_net) {
    delete slirp_net;
    slirp_net = nullptr;
  }
#endif
}

uint64_t virtio_net_t::get_device_features() const {
  return VIRTIO_F_VERSION_1 | VIRTIO_F_INDIRECT_DESC | VIRTIO_F_EVENT_IDX |
         VIRTIO_NET_F_MAC | VIRTIO_NET_F_STATUS | VIRTIO_NET_F_MTU |
         VIRTIO_NET_F_MRG_RXBUF; // Support merged receive buffers
}

void virtio_net_t::handle_driver_features(uint64_t features) {
  negotiated_features = features;

  // Enable EVENT_IDX on all queues if negotiated
  bool event_idx = (features & VIRTIO_F_EVENT_IDX) != 0;
  for (auto &queue : queues) {
    queue.set_event_idx(event_idx);
  }

  static bool debug_virtio = getenv("DEBUG_VIRTIO") != nullptr;
  if (debug_virtio || event_idx) {
    fprintf(stderr,
            "virtio-net: features negotiated=0x%lx, EVENT_IDX=%s\n",
            (unsigned long)features, event_idx ? "enabled" : "disabled");
  }
}

void virtio_net_t::handle_queue_notify(uint32_t queue_idx) {
  static bool debug_virtio = getenv("DEBUG_VIRTIO") != nullptr;
  if (debug_virtio) {
    fprintf(stderr, "virtio-net: queue_notify queue=%d\n", queue_idx);
  }
  if (queue_idx == QUEUE_TX) {
    process_tx_queue();
  } else if (queue_idx == QUEUE_RX) {
    process_rx_queue();
  }
}

bool virtio_net_t::read_config(reg_t offset, size_t len, uint8_t *bytes) {
  static bool debug_virtio = getenv("DEBUG_VIRTIO") != nullptr;

  if (offset + len > sizeof(config))
    return false;

  memcpy(bytes, (uint8_t *)&config + offset, len);

  if (debug_virtio) {
    fprintf(stderr, "virtio-net: read config offset=%lu len=%zu\n",
            (unsigned long)offset, len);
  }
  return true;
}

bool virtio_net_t::write_config(reg_t offset, size_t len,
                                const uint8_t *bytes) {
  // Most config is read-only, but allow MAC address writes
  if (offset < 6 && offset + len <= 6) {
    memcpy(config.mac + offset, bytes, len);
    return true;
  }
  return true;
}

void virtio_net_t::device_reset() {
  // Clear RX queue
  while (!rx_queue.empty())
    rx_queue.pop();
  negotiated_features = 0;
}

void virtio_net_t::tick(reg_t rtc_ticks) {
  static bool debug_virtio = getenv("DEBUG_VIRTIO") != nullptr;
  tick_count += rtc_ticks;

  // TAP mode - simple polling, no SLIRP complexity
  if (tap_net) {
    if (tick_count >= POLL_INTERVAL) {
      tick_count = 0;
      tap_net->poll(0); // Non-blocking poll

      // Process RX queue if we have packets
      if (!rx_queue.empty()) {
        process_rx_queue();
      }

      // Also poll TX queue - kernel may not always notify
      if (QUEUE_TX < queues.size() && queues[QUEUE_TX].is_ready() &&
          queues[QUEUE_TX].has_pending()) {
        process_tx_queue();
      }
    }
  }

#ifdef HAVE_SLIRP
  if (slirp_net) {
    // Poll immediately if:
    // - SLIRP requested attention via notify callback
    // - We have burst polls remaining (after FIN)
    // - We have packets waiting in rx_queue (need to deliver them)
    // - Regular interval reached
    bool should_poll = (tick_count >= POLL_INTERVAL) ||
                       slirp_net->has_notify_pending() ||
                       burst_poll_remaining > 0 ||
                       !rx_queue.empty();

    if (should_poll) {
      tick_count = 0;
      slirp_net->clear_notify_pending();

      // If we have burst polls remaining, decrement counter
      if (burst_poll_remaining > 0) {
        burst_poll_remaining--;
      }

      slirp_net->poll(0); // Non-blocking poll

      // Always try to process RX queue after polling SLIRP
      // This ensures any packets SLIRP just delivered from its internal buffer
      // get processed immediately, avoiding deadlock when SLIRP was backpressured
      process_rx_queue();

      // Also poll TX queue - kernel may not always notify
      if (QUEUE_TX < queues.size() && queues[QUEUE_TX].is_ready() &&
          queues[QUEUE_TX].has_pending()) {
        if (debug_virtio) {
          fprintf(stderr, "virtio-net: tick found TX pending, processing\n");
        }
        process_tx_queue();
      }
    }
  }
#endif
}

void virtio_net_t::process_tx_queue() {
  static bool debug_virtio = getenv("DEBUG_VIRTIO") != nullptr;
  static uint64_t tx_total = 0;

  if (QUEUE_TX >= queues.size() || !queues[QUEUE_TX].is_ready()) {
    if (debug_virtio)
      fprintf(stderr, "virtio-net: TX queue not ready\n");
    return;
  }

  std::vector<vring_desc> out_descs, in_descs;
  bool processed = false;

  while (queues[QUEUE_TX].has_pending()) {
    tx_total++;
    int head = queues[QUEUE_TX].get_avail_buf(out_descs, in_descs);
    if (head < 0)
      break;

    // Collect all data from descriptors (includes VirtIO net header)
    std::vector<uint8_t> tx_data;
    for (auto &desc : out_descs) {
      size_t old_size = tx_data.size();
      tx_data.resize(old_size + desc.len);
      queues[QUEUE_TX].read_guest_mem(desc.addr, tx_data.data() + old_size,
                                      desc.len);
    }

    size_t hdr_size = get_hdr_size();
    size_t eth_frame_size =
        tx_data.size() > hdr_size ? tx_data.size() - hdr_size : 0;

    if (debug_virtio) {
      fprintf(stderr,
              "virtio-net: TX packet size=%zu (hdr=%zu, eth frame=%zu)\n",
              tx_data.size(), hdr_size, eth_frame_size);
      // Print Ethernet header details
      if (eth_frame_size >= 14) {
        const uint8_t *eth = tx_data.data() + hdr_size;
        fprintf(stderr, "  TX dst MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", eth[0],
                eth[1], eth[2], eth[3], eth[4], eth[5]);
        fprintf(stderr, "  TX src MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", eth[6],
                eth[7], eth[8], eth[9], eth[10], eth[11]);
        fprintf(stderr, "  TX ethertype: %02x%02x\n", eth[12], eth[13]);
      }
    }

    // Check for TCP FIN/RST to trigger burst polling for connection closure
    bool is_tcp_fin = false;
    if (eth_frame_size >= 40) {
      const uint8_t *eth = tx_data.data() + hdr_size;
      if (eth[12] == 0x08 && eth[13] == 0x00) { // IPv4
        uint8_t ip_proto = eth[23];
        if (ip_proto == 6) { // TCP
          size_t ip_hdr_len = (eth[14] & 0x0f) * 4;
          if (eth_frame_size >= 14 + ip_hdr_len + 14) {
            uint8_t tcp_flags = eth[14 + ip_hdr_len + 13];
            if (tcp_flags & 0x01) { // FIN
              fprintf(stderr, "virtio-net: TX TCP FIN! flags=0x%02x\n",
                      tcp_flags);
              is_tcp_fin = true;
            }
            if (tcp_flags & 0x04) { // RST
              fprintf(stderr, "virtio-net: TX TCP RST! flags=0x%02x\n",
                      tcp_flags);
            }
          }
        }
      }
    }

    // Skip VirtIO net header and send raw Ethernet frame to network backend
    if (tx_data.size() > hdr_size) {
      // TAP mode - send directly to TAP device
      if (tap_net) {
        tap_net->send_packet(tx_data.data() + hdr_size,
                             tx_data.size() - hdr_size);
      }
#ifdef HAVE_SLIRP
      // SLIRP mode - send through SLIRP NAT
      else if (slirp_net) {
        static uint64_t tx_total = 0;
        static uint64_t last_tx_msg = 0;
        tx_total++;

        if (tx_total - last_tx_msg >= 100 || tx_total == 1) {
          fprintf(stderr,
                  "virtio-net: TX packet through SLIRP len=%zu total=%lu\n",
                  tx_data.size() - hdr_size, tx_total);
          last_tx_msg = tx_total;
        }

        slirp_net->send_packet(tx_data.data() + hdr_size,
                               tx_data.size() - hdr_size);

        // After sending FIN, schedule burst polling to help SLIRP complete TCP
        // state machine SLIRP needs multiple poll cycles to: process FIN ->
        // send ACK -> forward to socket We spread these polls across tick()
        // cycles to avoid blocking other devices
        if (is_tcp_fin) {
          burst_poll_remaining = 50; // Extra polls spread over next 50 ticks
        }
      }
#endif
    }

    // Return buffer to guest
    queues[QUEUE_TX].put_used_buf(head, 0);
    processed = true;
  }

  // Raise interrupt to notify guest (only if we processed something and driver
  // hasn't suppressed notifications via VRING_AVAIL_F_NO_INTERRUPT)
  if (processed && queues[QUEUE_TX].should_notify()) {
    raise_interrupt(VIRTIO_INT_USED_RING);
  }
}

void virtio_net_t::process_rx_queue() {
  static bool debug_virtio = getenv("DEBUG_VIRTIO") != nullptr;
  static uint64_t call_count = 0;
  static uint64_t last_consumed_msg = 0;
  call_count++;

  if (QUEUE_RX >= queues.size()) {

    if (call_count < 5 || call_count % 10000 == 0)
      fprintf(stderr, "virtio-net: process_rx_queue: QUEUE_RX out of range\n");
    return;
  }
  if (!queues[QUEUE_RX].is_ready()) {
    if (call_count < 5 || call_count % 10000 == 0)
      fprintf(stderr,
              "virtio-net: process_rx_queue: RX queue NOT ready (call #%lu)\n",
              call_count);
    // Drop the packet to avoid infinite loop in tick() - but only if we have one
    if (!rx_queue.empty()) {
      rx_queue.pop();
    }
    return;
  }

  // Queue is ready - log periodically
  if (call_count % 50000 == 0)
    fprintf(stderr,
            "virtio-net: process_rx_queue: RX queue ready, rx_queue.size=%zu, "
            "has_pending=%d (call #%lu)\n",
            rx_queue.size(), queues[QUEUE_RX].has_pending(), call_count);

  if (rx_queue.empty())
    return;

  // Have packets to deliver
  // fprintf(stderr, "virtio-net: process_rx_queue: %zu packets to deliver,
  // has_pending=%d\n",
  //         rx_queue.size(), queues[QUEUE_RX].has_pending());

  std::vector<vring_desc> out_descs, in_descs;
  bool processed = false;
  static uint64_t total_consumed = 0;

  while (queues[QUEUE_RX].has_pending() && !rx_queue.empty()) {
    total_consumed++;
    if (total_consumed - last_consumed_msg >= 500) {
      fprintf(stderr, "virtio-net: RX CONSUMED - total=%lu queue_size=%zu\n",
              total_consumed, rx_queue.size());
      last_consumed_msg = total_consumed;
    }

    int head = queues[QUEUE_RX].get_avail_buf(out_descs, in_descs);
    if (head < 0) {
      if (debug_virtio) {
        fprintf(stderr, "virtio-net: process_rx_queue: get_avail_buf failed\n");
      }
      // Failed to get buffer despite has_pending() being true.
      // Failed to get buffer despite has_pending() being true.
      // Likely memory error or inconsistency. Drop packet to avoid infinite
      // loop.
      rx_queue.pop();
      continue;
    }

    // Get next packet from RX queue
    rx_packet &pkt = rx_queue.front();

    if (debug_virtio) {
      fprintf(stderr,
              "virtio-net: process_rx_queue: delivering pkt len=%zu, "
              "in_descs=%zu\n",
              pkt.data.size(), in_descs.size());
      for (size_t i = 0; i < in_descs.size(); i++) {
        fprintf(stderr, "  in_desc[%zu]: addr=0x%lx len=%u flags=%u\n", i,
                (unsigned long)in_descs[i].addr, in_descs[i].len,
                in_descs[i].flags);
      }
    }

    // Write packet to guest buffers (includes VirtIO net header)
    uint32_t total_written = 0;
    size_t pkt_offset = 0;

    for (auto &desc : in_descs) {
      if (pkt_offset >= pkt.data.size())
        break;

      size_t write_len =
          std::min((size_t)desc.len, pkt.data.size() - pkt_offset);
      if (queues[QUEUE_RX].write_guest_mem(
              desc.addr, pkt.data.data() + pkt_offset, write_len)) {
        total_written += write_len;
        pkt_offset += write_len;
      }
    }

    if (debug_virtio) {
      fprintf(stderr, "virtio-net: process_rx_queue: wrote %u bytes to guest\n",
              total_written);
    }

    // Remove packet from queue
    rx_queue.pop();

    // Return buffer to guest with length
    queues[QUEUE_RX].put_used_buf(head, total_written);
    processed = true;
  }

  // Raise interrupt if we processed any packets and driver hasn't suppressed
  // notifications via VRING_AVAIL_F_NO_INTERRUPT
  if (processed && queues[QUEUE_RX].should_notify()) {
    raise_interrupt(VIRTIO_INT_USED_RING);
    if (debug_virtio) {
      fprintf(stderr, "virtio-net: process_rx_queue: raised interrupt\n");
    }
  }

  // If we still have packets but guest has no buffers, raise interrupt anyway
  // This helps wake up a stuck guest that might be waiting for notification
  // Only do this if driver hasn't suppressed notifications
  if (!rx_queue.empty() && !queues[QUEUE_RX].has_pending() &&
      queues[QUEUE_RX].should_notify()) {
    static uint64_t kick_count = 0;
    static uint64_t last_kick_msg = 0;
    kick_count++;
    if (kick_count - last_kick_msg >= 1000) {
      fprintf(stderr,
              "virtio-net: kicking guest - %zu packets waiting, no RX buffers "
              "(kick #%lu)\n",
              rx_queue.size(), kick_count);
      last_kick_msg = kick_count;
    }
    // Raise interrupt to nudge guest to provide more buffers
    raise_interrupt(VIRTIO_INT_USED_RING);
  }
}

// ============================================================================
// Device tree and factory functions
// ============================================================================

std::string
virtio_net_generate_dts(const sim_t *sim UNUSED,
                        const std::vector<std::string> &sargs UNUSED) {
  // Only generate DTS node if networking is enabled (SLIRP or TAP)
  if (!is_virtio_net_slirp_enabled() && !is_virtio_net_tap_enabled()) {
    return "";
  }

  std::stringstream s;
  reg_t base = VIRTIO_NET_BASE;
  reg_t sz = VIRTIO_NET_SIZE;

  fprintf(stderr, "virtio_net_generate_dts: generating DTS node at 0x%lx\n",
          (unsigned long)base);

  s << std::hex << "    virtio_net@" << base
    << " {\n"
       "      compatible = \"virtio,mmio\";\n"
       "      interrupt-parent = <&PLIC>;\n"
       "      interrupts = <"
    << std::dec << VIRTIO_NET_INTERRUPT_ID << ">;\n"
    << std::hex << "      reg = <0x" << (base >> 32) << " 0x"
    << (base & (uint32_t)-1) << " 0x" << (sz >> 32) << " 0x"
    << (sz & (uint32_t)-1)
    << ">;\n"
       "    };\n";
  return s.str();
}

int fdt_parse_virtio_net(const void *fdt, reg_t *addr, uint32_t *int_id,
                         const char *compatible) {
  int nodeoffset, len, rc;
  const fdt32_t *reg_p;

  nodeoffset = fdt_node_offset_by_compatible(fdt, -1, compatible);
  if (nodeoffset < 0)
    return nodeoffset;

  // Find node at our specific address
  while (nodeoffset >= 0) {
    reg_t node_addr;
    rc = fdt_get_node_addr_size(fdt, nodeoffset, &node_addr, NULL, "reg");
    if (rc >= 0 && node_addr == VIRTIO_NET_BASE) {
      if (addr)
        *addr = node_addr;
      reg_p = (fdt32_t *)fdt_getprop(fdt, nodeoffset, "interrupts", &len);
      if (int_id) {
        *int_id = reg_p ? fdt32_to_cpu(*reg_p) : VIRTIO_NET_INTERRUPT_ID;
      }
      return 0;
    }
    nodeoffset = fdt_node_offset_by_compatible(fdt, nodeoffset, compatible);
  }

  return -1;
}

virtio_net_t *virtio_net_parse_from_fdt(const void *fdt, const sim_t *sim,
                                        reg_t *base,
                                        const std::vector<std::string> &sargs) {
  uint32_t int_id;

  // Check if this is our virtio mmio device
  if (fdt_parse_virtio_net(fdt, base, &int_id, "virtio,mmio") != 0) {
    return nullptr;
  }

  abstract_interrupt_controller_t *intctrl = sim->get_intctrl();
  simif_t *simif = const_cast<simif_t *>(static_cast<const simif_t *>(sim));

  // TAP mode - direct layer 2 access to host network
  if (is_virtio_net_tap_enabled()) {
    const std::string &tap_name = get_virtio_net_tap_name();
    fprintf(
        stderr,
        "virtio_net_parse_from_fdt: creating TAP device (tap=%s) at 0x%lx\n",
        tap_name.c_str(), (unsigned long)*base);
    virtio_net_t *dev = new virtio_net_t(simif, intctrl, int_id, tap_name);
    return dev;
  }

#ifdef HAVE_SLIRP
  // SLIRP mode - NAT networking
  if (is_virtio_net_slirp_enabled()) {
    const virtio_net_slirp_config_t &slirp_config =
        get_virtio_net_slirp_config();
    fprintf(stderr,
            "virtio_net_parse_from_fdt: creating SLIRP device at 0x%lx\n",
            (unsigned long)*base);
    virtio_net_t *dev = new virtio_net_t(simif, intctrl, int_id, slirp_config);
    return dev;
  }
#endif

  return nullptr;
}

REGISTER_BUILTIN_DEVICE(virtio_net, virtio_net_parse_from_fdt,
                        virtio_net_generate_dts)
