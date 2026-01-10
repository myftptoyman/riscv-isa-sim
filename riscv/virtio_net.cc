// See LICENSE for license details.
#include "virtio_net.h"
#include "sim.h"
#include "dts.h"
#include "platform.h"
#include "libfdt.h"

#include <cstring>
#include <sstream>

// Global SLIRP configuration
static virtio_net_slirp_config_t g_virtio_net_slirp_config;

void set_virtio_net_slirp_config(const virtio_net_slirp_config_t& config) {
  fprintf(stderr, "set_virtio_net_slirp_config: enabled=%d, port_forwards=%zu\n",
          config.enabled, config.port_forwards.size());
  g_virtio_net_slirp_config = config;
}

const virtio_net_slirp_config_t& get_virtio_net_slirp_config() {
  return g_virtio_net_slirp_config;
}

bool is_virtio_net_slirp_enabled() {
  return g_virtio_net_slirp_config.enabled;
}

#ifdef HAVE_SLIRP

virtio_net_t::virtio_net_t(simif_t* sim,
                           abstract_interrupt_controller_t* intctrl,
                           uint32_t interrupt_id,
                           const virtio_net_slirp_config_t& slirp_config)
  : virtio_base_t(sim, intctrl, interrupt_id, NUM_QUEUES, VIRTIO_NET_QUEUE_SIZE),
    negotiated_features(0),
    tick_count(0),
    slirp_net(nullptr)
{
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

void virtio_net_t::init_slirp(const virtio_net_slirp_config_t& cfg) {
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
  slirp_net->set_rx_callback([this](const uint8_t* data, size_t len) {
    this->slirp_rx_callback(data, len);
  });

  fprintf(stderr, "virtio-net: SLIRP networking enabled (standard VirtIO-Net for Linux)\n");
  fprintf(stderr, "virtio-net: MAC address %02x:%02x:%02x:%02x:%02x:%02x\n",
          config.mac[0], config.mac[1], config.mac[2],
          config.mac[3], config.mac[4], config.mac[5]);

  // Print port forwarding info
  for (const auto& pf : cfg.port_forwards) {
    fprintf(stderr, "virtio-net: port forward host:%d -> guest:%d\n",
            pf.first, pf.second);
  }
}

void virtio_net_t::slirp_rx_callback(const uint8_t* data, size_t len) {
  static bool debug_virtio = getenv("DEBUG_VIRTIO") != nullptr;
  if (debug_virtio) {
    fprintf(stderr, "virtio-net: slirp_rx_callback len=%zu (packet from network to guest)\n", len);
    // Dump first 20 bytes of Ethernet frame (dst MAC, src MAC, ethertype)
    if (len >= 14) {
      fprintf(stderr, "  dst MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
              data[0], data[1], data[2], data[3], data[4], data[5]);
      fprintf(stderr, "  src MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
              data[6], data[7], data[8], data[9], data[10], data[11]);
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
  virtio_net_hdr* hdr = reinterpret_cast<virtio_net_hdr*>(pkt.data.data());
  hdr->flags = 0;
  hdr->gso_type = VIRTIO_NET_HDR_GSO_NONE;

  // If MRG_RXBUF is negotiated, set num_buffers to 1 (single buffer for this packet)
  if (negotiated_features & VIRTIO_NET_F_MRG_RXBUF) {
    // num_buffers is at offset 10 (after the 10-byte base header), little-endian
    uint16_t num_buffers = 1;
    pkt.data[10] = num_buffers & 0xff;
    pkt.data[11] = (num_buffers >> 8) & 0xff;
  }

  // Copy Ethernet frame after header
  memcpy(pkt.data.data() + hdr_size, data, len);

  // Queue packet for delivery to guest
  rx_queue.push(std::move(pkt));
  if (debug_virtio) {
    fprintf(stderr, "virtio-net: rx_queue.size()=%zu after push\n", rx_queue.size());
  }
}

#endif // HAVE_SLIRP

virtio_net_t::~virtio_net_t() {
#ifdef HAVE_SLIRP
  if (slirp_net) {
    delete slirp_net;
    slirp_net = nullptr;
  }
#endif
}

uint64_t virtio_net_t::get_device_features() const {
  return VIRTIO_F_VERSION_1 |
         VIRTIO_F_INDIRECT_DESC |
         VIRTIO_NET_F_MAC |
         VIRTIO_NET_F_STATUS |
         VIRTIO_NET_F_MTU |
         VIRTIO_NET_F_MRG_RXBUF;  // Support merged receive buffers
}

void virtio_net_t::handle_driver_features(uint64_t features) {
  negotiated_features = features;
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

bool virtio_net_t::read_config(reg_t offset, size_t len, uint8_t* bytes) {
  static bool debug_virtio = getenv("DEBUG_VIRTIO") != nullptr;

  if (offset + len > sizeof(config))
    return false;

  memcpy(bytes, (uint8_t*)&config + offset, len);

  if (debug_virtio) {
    fprintf(stderr, "virtio-net: read config offset=%lu len=%zu\n",
            (unsigned long)offset, len);
  }
  return true;
}

bool virtio_net_t::write_config(reg_t offset, size_t len, const uint8_t* bytes) {
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
  static int tick_debug_count = 0;
  tick_count += rtc_ticks;

#ifdef HAVE_SLIRP
  if (slirp_net) {
    if (tick_count >= POLL_INTERVAL) {
      tick_count = 0;
      slirp_net->poll(0);  // Non-blocking poll

      // Debug: check queue status periodically (only every 10000 ticks to reduce noise)
      if (debug_virtio && (++tick_debug_count % 10000) == 0) {
        bool rx_ready = QUEUE_RX < queues.size() && queues[QUEUE_RX].is_ready();
        bool tx_ready = QUEUE_TX < queues.size() && queues[QUEUE_TX].is_ready();
        fprintf(stderr, "virtio-net: tick - RX ready=%d, TX ready=%d, rx_queue=%zu\n",
                rx_ready, tx_ready, rx_queue.size());
      }

      // Process RX queue if we have packets
      if (!rx_queue.empty()) {
        process_rx_queue();
      }

      // Also poll TX queue - kernel may not always notify
      if (QUEUE_TX < queues.size() && queues[QUEUE_TX].is_ready() && queues[QUEUE_TX].has_pending()) {
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

  if (QUEUE_TX >= queues.size() || !queues[QUEUE_TX].is_ready()) {
    if (debug_virtio) fprintf(stderr, "virtio-net: TX queue not ready\n");
    return;
  }

  std::vector<vring_desc> out_descs, in_descs;
  bool processed = false;

  while (queues[QUEUE_TX].has_pending()) {
    int head = queues[QUEUE_TX].get_avail_buf(out_descs, in_descs);
    if (head < 0)
      break;

    // Collect all data from descriptors (includes VirtIO net header)
    std::vector<uint8_t> tx_data;
    for (auto& desc : out_descs) {
      size_t old_size = tx_data.size();
      tx_data.resize(old_size + desc.len);
      queues[QUEUE_TX].read_guest_mem(desc.addr, tx_data.data() + old_size, desc.len);
    }

    size_t hdr_size = get_hdr_size();
    if (debug_virtio) {
      size_t eth_frame_size = tx_data.size() > hdr_size ? tx_data.size() - hdr_size : 0;
      fprintf(stderr, "virtio-net: TX packet size=%zu (hdr=%zu, eth frame=%zu)\n",
              tx_data.size(), hdr_size, eth_frame_size);
      // Print Ethernet header details
      if (eth_frame_size >= 14) {
        const uint8_t* eth = tx_data.data() + hdr_size;
        fprintf(stderr, "  TX dst MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
                eth[0], eth[1], eth[2], eth[3], eth[4], eth[5]);
        fprintf(stderr, "  TX src MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
                eth[6], eth[7], eth[8], eth[9], eth[10], eth[11]);
        fprintf(stderr, "  TX ethertype: %02x%02x\n", eth[12], eth[13]);
      }
    }

    // Skip VirtIO net header and send raw Ethernet frame to SLIRP
    if (tx_data.size() > hdr_size) {
#ifdef HAVE_SLIRP
      if (slirp_net) {
        slirp_net->send_packet(tx_data.data() + hdr_size,
                               tx_data.size() - hdr_size);
      }
#endif
    }

    // Return buffer to guest
    queues[QUEUE_TX].put_used_buf(head, 0);
    processed = true;
  }

  // Raise interrupt to notify guest (only if we processed something)
  if (processed) {
    raise_interrupt(VIRTIO_INT_USED_RING);
  }
}

void virtio_net_t::process_rx_queue() {
  static bool debug_virtio = getenv("DEBUG_VIRTIO") != nullptr;

  if (QUEUE_RX >= queues.size() || !queues[QUEUE_RX].is_ready())
    return;

  if (rx_queue.empty())
    return;

  std::vector<vring_desc> out_descs, in_descs;
  bool processed = false;

  while (queues[QUEUE_RX].has_pending() && !rx_queue.empty()) {
    int head = queues[QUEUE_RX].get_avail_buf(out_descs, in_descs);
    if (head < 0) {
      if (debug_virtio) {
        fprintf(stderr, "virtio-net: process_rx_queue: get_avail_buf failed\n");
      }
      break;
    }

    // Get next packet from RX queue
    rx_packet& pkt = rx_queue.front();

    if (debug_virtio) {
      fprintf(stderr, "virtio-net: process_rx_queue: delivering pkt len=%zu, in_descs=%zu\n",
              pkt.data.size(), in_descs.size());
      for (size_t i = 0; i < in_descs.size(); i++) {
        fprintf(stderr, "  in_desc[%zu]: addr=0x%lx len=%u flags=%u\n",
                i, (unsigned long)in_descs[i].addr, in_descs[i].len, in_descs[i].flags);
      }
    }

    // Write packet to guest buffers (includes VirtIO net header)
    uint32_t total_written = 0;
    size_t pkt_offset = 0;

    for (auto& desc : in_descs) {
      if (pkt_offset >= pkt.data.size())
        break;

      size_t write_len = std::min((size_t)desc.len, pkt.data.size() - pkt_offset);
      if (queues[QUEUE_RX].write_guest_mem(desc.addr, pkt.data.data() + pkt_offset, write_len)) {
        total_written += write_len;
        pkt_offset += write_len;
      }
    }

    if (debug_virtio) {
      fprintf(stderr, "virtio-net: process_rx_queue: wrote %u bytes to guest\n", total_written);
    }

    // Remove packet from queue
    rx_queue.pop();

    // Return buffer to guest with length
    queues[QUEUE_RX].put_used_buf(head, total_written);
    processed = true;
  }

  // Raise interrupt if we processed any packets
  if (processed) {
    raise_interrupt(VIRTIO_INT_USED_RING);
    if (debug_virtio) {
      fprintf(stderr, "virtio-net: process_rx_queue: raised interrupt\n");
    }
  }
}

// ============================================================================
// Device tree and factory functions
// ============================================================================

std::string virtio_net_generate_dts(const sim_t* sim UNUSED, const std::vector<std::string>& sargs UNUSED)
{
  // Only generate DTS node if SLIRP is enabled
  if (!is_virtio_net_slirp_enabled()) {
    return "";
  }

  std::stringstream s;
  reg_t base = VIRTIO_NET_BASE;
  reg_t sz = VIRTIO_NET_SIZE;

  fprintf(stderr, "virtio_net_generate_dts: generating DTS node at 0x%lx\n", (unsigned long)base);

  s << std::hex
    << "    virtio_net@" << base << " {\n"
       "      compatible = \"virtio,mmio\";\n"
       "      interrupt-parent = <&PLIC>;\n"
       "      interrupts = <" << std::dec << VIRTIO_NET_INTERRUPT_ID << ">;\n"
    << std::hex
    << "      reg = <0x" << (base >> 32) << " 0x" << (base & (uint32_t)-1)
    <<              " 0x" << (sz >> 32) << " 0x" << (sz & (uint32_t)-1) << ">;\n"
       "    };\n";
  return s.str();
}

int fdt_parse_virtio_net(const void *fdt, reg_t *addr, uint32_t *int_id,
                          const char *compatible)
{
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
      if (addr) *addr = node_addr;
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

virtio_net_t* virtio_net_parse_from_fdt(const void* fdt, const sim_t* sim,
                                         reg_t* base,
                                         const std::vector<std::string>& sargs)
{
  uint32_t int_id;

  // Check if this is our virtio mmio device
  if (fdt_parse_virtio_net(fdt, base, &int_id, "virtio,mmio") != 0) {
    return nullptr;
  }

#ifdef HAVE_SLIRP
  // Check if SLIRP mode is enabled
  if (!is_virtio_net_slirp_enabled()) {
    return nullptr;
  }

  abstract_interrupt_controller_t* intctrl = sim->get_intctrl();
  simif_t* simif = const_cast<simif_t*>(static_cast<const simif_t*>(sim));

  const virtio_net_slirp_config_t& slirp_config = get_virtio_net_slirp_config();
  virtio_net_t* dev = new virtio_net_t(simif, intctrl, int_id, slirp_config);
  return dev;
#else
  return nullptr;
#endif
}

REGISTER_BUILTIN_DEVICE(virtio_net, virtio_net_parse_from_fdt, virtio_net_generate_dts)
