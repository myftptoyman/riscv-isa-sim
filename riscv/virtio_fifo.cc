// See LICENSE for license details.
#include "virtio_fifo.h"
#include "sim.h"
#include "dts.h"
#include "platform.h"
#include "libfdt.h"

#include <sys/socket.h>
#include <sys/un.h>
#include <sys/poll.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <cstring>
#include <sstream>

// Global socket path for this device (set via command line)
static std::string g_virtio_fifo_socket_path;

// Global SLIRP configuration
static virtio_fifo_slirp_config_t g_virtio_fifo_slirp_config;

void set_virtio_fifo_socket_path(const std::string& path) {
  g_virtio_fifo_socket_path = path;
}

const std::string& get_virtio_fifo_socket_path() {
  return g_virtio_fifo_socket_path;
}

void set_virtio_fifo_slirp_config(const virtio_fifo_slirp_config_t& config) {
  g_virtio_fifo_slirp_config = config;
}

const virtio_fifo_slirp_config_t& get_virtio_fifo_slirp_config() {
  return g_virtio_fifo_slirp_config;
}

bool is_virtio_fifo_slirp_enabled() {
  return g_virtio_fifo_slirp_config.enabled;
}

virtio_fifo_t::virtio_fifo_t(simif_t* sim,
                             abstract_interrupt_controller_t* intctrl,
                             uint32_t interrupt_id,
                             const std::string& socket_path)
  : virtio_base_t(sim, intctrl, interrupt_id, NUM_QUEUES, VIRTIO_FIFO_QUEUE_SIZE),
    socket_path(socket_path),
    listen_fd(-1),
    conn_fd(-1),
    connected(false),
    tick_count(0)
#ifdef HAVE_SLIRP
    , use_slirp(false),
    slirp_net(nullptr)
#endif
{
  // Initialize configuration
  config.max_tx_size = VIRTIO_FIFO_MAX_TX_SIZE;
  config.max_rx_size = VIRTIO_FIFO_MAX_RX_SIZE;
  config.flags = 0;

  // Initialize socket if path provided
  if (!socket_path.empty()) {
    init_socket();
  }
}

#ifdef HAVE_SLIRP
virtio_fifo_t::virtio_fifo_t(simif_t* sim,
                             abstract_interrupt_controller_t* intctrl,
                             uint32_t interrupt_id,
                             const virtio_fifo_slirp_config_t& slirp_config)
  : virtio_base_t(sim, intctrl, interrupt_id, NUM_QUEUES, VIRTIO_FIFO_QUEUE_SIZE),
    socket_path(""),
    listen_fd(-1),
    conn_fd(-1),
    connected(false),
    tick_count(0),
    use_slirp(true),
    slirp_net(nullptr)
{
  // Initialize configuration
  config.max_tx_size = VIRTIO_FIFO_MAX_TX_SIZE;
  config.max_rx_size = VIRTIO_FIFO_MAX_RX_SIZE;
  config.flags = 0;

  // Initialize SLIRP
  init_slirp(slirp_config);
}

void virtio_fifo_t::init_slirp(const virtio_fifo_slirp_config_t& cfg) {
  slirp_config_t slirp_cfg;
  slirp_cfg.host_port = cfg.host_port;
  slirp_cfg.guest_port = cfg.guest_port;
  slirp_cfg.debug = cfg.debug;

  slirp_net = new slirp_net_t(slirp_cfg);
  if (!slirp_net->init()) {
    delete slirp_net;
    slirp_net = nullptr;
    fprintf(stderr, "virtio-fifo: failed to initialize SLIRP\n");
    return;
  }

  // Set up receive callback
  slirp_net->set_rx_callback([this](const uint8_t* data, size_t len) {
    this->slirp_rx_callback(data, len);
  });

  fprintf(stderr, "virtio-fifo: SLIRP networking enabled\n");
  fprintf(stderr, "virtio-fifo: access web server at http://localhost:%d\n", cfg.host_port);
}

void virtio_fifo_t::slirp_rx_callback(const uint8_t* data, size_t len) {
  // Add received packet to RX buffer with length prefix
  // Format: 2-byte big-endian length + data
  if (len > 0 && len <= VIRTIO_FIFO_MAX_RX_SIZE) {
    rx_buffer.push((len >> 8) & 0xFF);
    rx_buffer.push(len & 0xFF);
    for (size_t i = 0; i < len; i++) {
      rx_buffer.push(data[i]);
    }
  }
}
#endif

virtio_fifo_t::~virtio_fifo_t() {
  close_socket();
#ifdef HAVE_SLIRP
  if (slirp_net) {
    delete slirp_net;
    slirp_net = nullptr;
  }
#endif
}

bool virtio_fifo_t::init_socket() {
  // Create Unix domain socket
  listen_fd = socket(AF_UNIX, SOCK_STREAM, 0);
  if (listen_fd < 0) {
    fprintf(stderr, "virtio-fifo: failed to create socket: %s\n", strerror(errno));
    return false;
  }

  // Set non-blocking
  int flags = fcntl(listen_fd, F_GETFL, 0);
  fcntl(listen_fd, F_SETFL, flags | O_NONBLOCK);

  // Bind to path
  struct sockaddr_un addr;
  memset(&addr, 0, sizeof(addr));
  addr.sun_family = AF_UNIX;
  strncpy(addr.sun_path, socket_path.c_str(), sizeof(addr.sun_path) - 1);

  // Remove existing socket file
  unlink(socket_path.c_str());

  if (bind(listen_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    fprintf(stderr, "virtio-fifo: failed to bind socket to %s: %s\n",
            socket_path.c_str(), strerror(errno));
    close(listen_fd);
    listen_fd = -1;
    return false;
  }

  if (listen(listen_fd, 1) < 0) {
    fprintf(stderr, "virtio-fifo: failed to listen on socket: %s\n", strerror(errno));
    close(listen_fd);
    listen_fd = -1;
    return false;
  }

  fprintf(stderr, "virtio-fifo: listening on %s\n", socket_path.c_str());
  return true;
}

void virtio_fifo_t::close_socket() {
  if (conn_fd >= 0) {
    close(conn_fd);
    conn_fd = -1;
  }
  if (listen_fd >= 0) {
    close(listen_fd);
    listen_fd = -1;
  }
  if (!socket_path.empty()) {
    unlink(socket_path.c_str());
  }
  connected = false;
}

bool virtio_fifo_t::accept_connection() {
  if (listen_fd < 0 || connected)
    return connected;

  struct sockaddr_un addr;
  socklen_t addr_len = sizeof(addr);
  int fd = accept(listen_fd, (struct sockaddr*)&addr, &addr_len);
  if (fd < 0) {
    if (errno != EAGAIN && errno != EWOULDBLOCK) {
      fprintf(stderr, "virtio-fifo: accept failed: %s\n", strerror(errno));
    }
    return false;
  }

  // Set non-blocking
  int flags = fcntl(fd, F_GETFL, 0);
  fcntl(fd, F_SETFL, flags | O_NONBLOCK);

  conn_fd = fd;
  connected = true;
  fprintf(stderr, "virtio-fifo: client connected\n");
  return true;
}

bool virtio_fifo_t::has_rx_data() {
  if (!connected || conn_fd < 0)
    return !rx_buffer.empty();

  struct pollfd pfd;
  pfd.fd = conn_fd;
  pfd.events = POLLIN;
  pfd.revents = 0;

  int ret = poll(&pfd, 1, 0);
  return (ret > 0 && (pfd.revents & POLLIN)) || !rx_buffer.empty();
}

ssize_t virtio_fifo_t::receive_data(void* buf, size_t max_len) {
  if (!connected || conn_fd < 0)
    return 0;

  ssize_t n = recv(conn_fd, buf, max_len, MSG_DONTWAIT);
  if (n == 0) {
    // Connection closed
    fprintf(stderr, "virtio-fifo: client disconnected\n");
    close(conn_fd);
    conn_fd = -1;
    connected = false;
    return 0;
  } else if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK)
      return 0;
    fprintf(stderr, "virtio-fifo: recv error: %s\n", strerror(errno));
    close(conn_fd);
    conn_fd = -1;
    connected = false;
    return -1;
  }
  return n;
}

ssize_t virtio_fifo_t::send_data(const void* buf, size_t len) {
  if (!connected || conn_fd < 0)
    return -1;

  ssize_t n = send(conn_fd, buf, len, MSG_NOSIGNAL);
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK)
      return 0;
    fprintf(stderr, "virtio-fifo: send error: %s\n", strerror(errno));
    close(conn_fd);
    conn_fd = -1;
    connected = false;
    return -1;
  }
  return n;
}

uint64_t virtio_fifo_t::get_device_features() const {
  return VIRTIO_F_VERSION_1;
}

void virtio_fifo_t::handle_driver_features(uint64_t features) {
  // Store negotiated features if needed
  (void)features;
}

void virtio_fifo_t::handle_queue_notify(uint32_t queue_idx) {
  if (queue_idx == QUEUE_TX) {
    process_tx_queue();
  } else if (queue_idx == QUEUE_RX) {
    process_rx_queue();
  }
}

bool virtio_fifo_t::read_config(reg_t offset, size_t len, uint8_t* bytes) {
  if (offset + len > sizeof(config))
    return false;

  memcpy(bytes, (uint8_t*)&config + offset, len);
  return true;
}

bool virtio_fifo_t::write_config(reg_t offset, size_t len, const uint8_t* bytes) {
  // Config is read-only for now
  (void)offset;
  (void)len;
  (void)bytes;
  return true;
}

void virtio_fifo_t::device_reset() {
  // Clear RX buffer
  while (!rx_buffer.empty())
    rx_buffer.pop();
}

void virtio_fifo_t::tick(reg_t rtc_ticks) {
  tick_count += rtc_ticks;

#ifdef HAVE_SLIRP
  // SLIRP mode: poll SLIRP network
  if (use_slirp && slirp_net) {
    if (tick_count >= POLL_INTERVAL) {
      tick_count = 0;
      slirp_net->poll(0);  // Non-blocking poll
      // Process RX queue if we have data
      if (!rx_buffer.empty()) {
        process_rx_queue();
      }
    }
    return;
  }
#endif

  // Socket mode: check for new connections periodically
  if (!connected && listen_fd >= 0) {
    accept_connection();
  }

  // Poll for incoming data and process RX queue periodically
  if (tick_count >= POLL_INTERVAL) {
    tick_count = 0;

    // Read data from socket into buffer
    if (connected && conn_fd >= 0) {
      uint8_t tmp[1024];
      ssize_t n = receive_data(tmp, sizeof(tmp));
      if (n > 0) {
        for (ssize_t i = 0; i < n; i++) {
          rx_buffer.push(tmp[i]);
        }
        // Try to fill RX queue with available data
        process_rx_queue();
      }
    }
  }
}

void virtio_fifo_t::process_tx_queue() {
  if (QUEUE_TX >= queues.size() || !queues[QUEUE_TX].is_ready())
    return;

  std::vector<vring_desc> out_descs, in_descs;

  while (queues[QUEUE_TX].has_pending()) {
    int head = queues[QUEUE_TX].get_avail_buf(out_descs, in_descs);
    if (head < 0)
      break;

    // Process TX: read data from guest and send to host/SLIRP
    uint32_t total_len = 0;

    // Collect all data from descriptors
    std::vector<uint8_t> tx_data;
    for (auto& desc : out_descs) {
      size_t old_size = tx_data.size();
      tx_data.resize(old_size + desc.len);
      if (queues[QUEUE_TX].read_guest_mem(desc.addr, tx_data.data() + old_size, desc.len)) {
        total_len += desc.len;
      }
    }

#ifdef HAVE_SLIRP
    if (use_slirp && slirp_net && tx_data.size() >= 2) {
      // Parse frames from TX data (format: 2-byte length + frame)
      size_t offset = 0;
      while (offset + 2 <= tx_data.size()) {
        uint16_t frame_len = (tx_data[offset] << 8) | tx_data[offset + 1];
        if (frame_len == 0 || offset + 2 + frame_len > tx_data.size())
          break;
        // Send Ethernet frame to SLIRP
        slirp_net->send_packet(tx_data.data() + offset + 2, frame_len);
        offset += 2 + frame_len;
      }
    } else
#endif
    {
      // Socket mode: send data directly
      if (tx_data.size() > 0) {
        send_data(tx_data.data(), tx_data.size());
      }
    }

    // Return buffer to guest
    queues[QUEUE_TX].put_used_buf(head, total_len);
  }

  // Raise interrupt to notify guest
  raise_interrupt(VIRTIO_INT_USED_RING);
}

void virtio_fifo_t::process_rx_queue() {
  if (QUEUE_RX >= queues.size() || !queues[QUEUE_RX].is_ready())
    return;

  if (rx_buffer.empty())
    return;

  std::vector<vring_desc> out_descs, in_descs;
  bool processed = false;

  while (queues[QUEUE_RX].has_pending() && !rx_buffer.empty()) {
    int head = queues[QUEUE_RX].get_avail_buf(out_descs, in_descs);
    if (head < 0)
      break;

    // Process RX: write data from host to guest
    uint32_t total_len = 0;
    for (auto& desc : in_descs) {
      if (rx_buffer.empty())
        break;

      uint32_t write_len = std::min((uint32_t)desc.len, (uint32_t)rx_buffer.size());
      std::vector<uint8_t> buf(write_len);
      for (uint32_t i = 0; i < write_len; i++) {
        buf[i] = rx_buffer.front();
        rx_buffer.pop();
      }

      if (queues[QUEUE_RX].write_guest_mem(desc.addr, buf.data(), write_len)) {
        total_len += write_len;
      }
    }

    // Return buffer to guest
    queues[QUEUE_RX].put_used_buf(head, total_len);
    processed = true;
  }

  // Raise interrupt if we processed any buffers
  if (processed) {
    raise_interrupt(VIRTIO_INT_USED_RING);
  }
}

// ============================================================================
// Device tree and factory functions
// ============================================================================

std::string virtio_fifo_generate_dts(const sim_t* sim, const std::vector<std::string>& sargs UNUSED)
{
  // Only generate DTS if device is configured (socket path or SLIRP enabled)
  const std::string& socket_path = get_virtio_fifo_socket_path();
  const virtio_fifo_slirp_config_t& slirp_config = get_virtio_fifo_slirp_config();

  if (socket_path.empty() && !slirp_config.enabled)
    return "";  // No device configured, don't add to DTS

  std::stringstream s;
  reg_t base = VIRTIO_FIFO_BASE;
  reg_t sz = VIRTIO_FIFO_SIZE;

  s << std::hex
    << "    virtio_fifo@" << base << " {\n"
       "      compatible = \"virtio,mmio\";\n"
       "      interrupt-parent = <&PLIC>;\n"
       "      interrupts = <" << std::dec << VIRTIO_FIFO_INTERRUPT_ID << ">;\n"
    << std::hex
    << "      reg = <0x" << (base >> 32) << " 0x" << (base & (uint32_t)-1)
    <<              " 0x" << (sz >> 32) << " 0x" << (sz & (uint32_t)-1) << ">;\n"
       "    };\n";
  return s.str();
}

int fdt_parse_virtio_fifo(const void *fdt, reg_t *addr, uint32_t *int_id,
                          const char *compatible)
{
  int nodeoffset, len, rc;
  const fdt32_t *reg_p;

  nodeoffset = fdt_node_offset_by_compatible(fdt, -1, compatible);
  if (nodeoffset < 0)
    return nodeoffset;

  rc = fdt_get_node_addr_size(fdt, nodeoffset, addr, NULL, "reg");
  if (rc < 0 || !addr)
    return -ENODEV;

  reg_p = (fdt32_t *)fdt_getprop(fdt, nodeoffset, "interrupts", &len);
  if (int_id) {
    if (reg_p) {
      *int_id = fdt32_to_cpu(*reg_p);
    } else {
      *int_id = VIRTIO_FIFO_INTERRUPT_ID;
    }
  }

  return 0;
}

virtio_fifo_t* virtio_fifo_parse_from_fdt(const void* fdt, const sim_t* sim,
                                          reg_t* base,
                                          const std::vector<std::string>& sargs)
{
  uint32_t int_id;

  // Check if this is a virtio mmio device
  if (fdt_parse_virtio_fifo(fdt, base, &int_id, "virtio,mmio") != 0)
    return nullptr;

  abstract_interrupt_controller_t* intctrl = sim->get_intctrl();
  simif_t* simif = const_cast<simif_t*>(static_cast<const simif_t*>(sim));

#ifdef HAVE_SLIRP
  // Check for SLIRP mode first
  if (is_virtio_fifo_slirp_enabled()) {
    const virtio_fifo_slirp_config_t& slirp_config = get_virtio_fifo_slirp_config();
    return new virtio_fifo_t(simif, intctrl, int_id, slirp_config);
  }
#endif

  // Socket mode
  const std::string& socket_path = get_virtio_fifo_socket_path();
  if (socket_path.empty())
    return nullptr;

  return new virtio_fifo_t(simif, intctrl, int_id, socket_path);
}

REGISTER_BUILTIN_DEVICE(virtio_fifo, virtio_fifo_parse_from_fdt, virtio_fifo_generate_dts)
