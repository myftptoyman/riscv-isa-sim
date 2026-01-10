// See LICENSE for license details.
#include "virtio_block.h"
#include "sim.h"
#include "dts.h"
#include "platform.h"
#include "libfdt.h"

#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <cstring>
#include <sstream>

// Global configuration for this device (set via command line)
static std::string g_virtio_block_image_path;
static bool g_virtio_block_readonly = false;

void set_virtio_block_image_path(const std::string& path) {
  g_virtio_block_image_path = path;
}

const std::string& get_virtio_block_image_path() {
  return g_virtio_block_image_path;
}

void set_virtio_block_readonly(bool readonly) {
  g_virtio_block_readonly = readonly;
}

bool get_virtio_block_readonly() {
  return g_virtio_block_readonly;
}

bool is_virtio_block_enabled() {
  return !g_virtio_block_image_path.empty();
}

virtio_block_t::virtio_block_t(simif_t* sim,
                               abstract_interrupt_controller_t* intctrl,
                               uint32_t interrupt_id,
                               const std::string& image_path,
                               bool readonly)
  : virtio_base_t(sim, intctrl, interrupt_id, NUM_QUEUES, VIRTIO_BLK_QUEUE_SIZE),
    image_path(image_path),
    device_id("spike-virtio-blk"),
    fd(-1),
    readonly(readonly),
    file_size(0),
    tick_count(0)
{
  // Initialize configuration
  memset(&config, 0, sizeof(config));
  config.blk_size = VIRTIO_BLK_SECTOR_SIZE;
  config.size_max = 65536;  // 64KB max segment
  config.seg_max = 128;     // Max segments per request

  // Open image file
  int flags = readonly ? O_RDONLY : O_RDWR;
  fd = open(image_path.c_str(), flags);
  if (fd < 0) {
    fprintf(stderr, "virtio-block: failed to open %s: %s\n",
            image_path.c_str(), strerror(errno));
    return;
  }

  // Get file size
  struct stat st;
  if (fstat(fd, &st) < 0) {
    fprintf(stderr, "virtio-block: failed to stat %s: %s\n",
            image_path.c_str(), strerror(errno));
    close(fd);
    fd = -1;
    return;
  }

  file_size = st.st_size;
  config.capacity = file_size / VIRTIO_BLK_SECTOR_SIZE;

  fprintf(stderr, "virtio-block: %s opened, %lu sectors (%lu bytes)%s\n",
          image_path.c_str(),
          (unsigned long)config.capacity,
          (unsigned long)file_size,
          readonly ? " [read-only]" : "");
}

virtio_block_t::~virtio_block_t() {
  if (fd >= 0) {
    close(fd);
    fd = -1;
  }
}

uint64_t virtio_block_t::get_device_features() const {
  uint64_t features = VIRTIO_F_VERSION_1;

  // Support indirect descriptors (allows unlimited scatter-gather)
  features |= VIRTIO_F_INDIRECT_DESC;

  // Advertise supported features
  features |= VIRTIO_BLK_F_SIZE_MAX;
  features |= VIRTIO_BLK_F_SEG_MAX;
  features |= VIRTIO_BLK_F_BLK_SIZE;
  features |= VIRTIO_BLK_F_FLUSH;

  if (readonly) {
    features |= VIRTIO_BLK_F_RO;
  }

  return features;
}

void virtio_block_t::handle_driver_features(uint64_t features) {
  // Store negotiated features if needed
  (void)features;
}

void virtio_block_t::handle_queue_notify(uint32_t queue_idx) {
  if (queue_idx == QUEUE_REQUEST) {
    process_request_queue();
  }
}

bool virtio_block_t::read_config(reg_t offset, size_t len, uint8_t* bytes) {
  if (offset + len > sizeof(config))
    return false;

  memcpy(bytes, (uint8_t*)&config + offset, len);
  return true;
}

bool virtio_block_t::write_config(reg_t offset, size_t len, const uint8_t* bytes) {
  // Block device config is read-only
  (void)offset;
  (void)len;
  (void)bytes;
  return true;
}

void virtio_block_t::device_reset() {
  // Nothing to reset for block device
}

void virtio_block_t::tick(reg_t rtc_ticks) {
  tick_count += rtc_ticks;

  // Nothing to poll for block device (synchronous I/O)
  (void)tick_count;
}

void virtio_block_t::process_request_queue() {
  if (QUEUE_REQUEST >= queues.size() || !queues[QUEUE_REQUEST].is_ready())
    return;

  if (fd < 0)
    return;

  std::vector<vring_desc> out_descs, in_descs;

  while (queues[QUEUE_REQUEST].has_pending()) {
    int head = queues[QUEUE_REQUEST].get_avail_buf(out_descs, in_descs);
    if (head < 0)
      break;

    // Block request format:
    // out_descs[0]: request header (virtio_blk_req)
    // out_descs[1...n]: data for write operations
    // in_descs[0...m-1]: data for read operations
    // in_descs[m]: status byte (1 byte)

    if (out_descs.empty() || in_descs.empty()) {
      // Invalid request - need at least header and status
      queues[QUEUE_REQUEST].put_used_buf(head, 0);
      continue;
    }

    // Read request header
    virtio_blk_req req;
    if (!queues[QUEUE_REQUEST].read_guest_mem(out_descs[0].addr, &req, sizeof(req))) {
      queues[QUEUE_REQUEST].put_used_buf(head, 0);
      continue;
    }

    uint8_t status = VIRTIO_BLK_S_OK;
    uint32_t total_len = 0;

    switch (req.type) {
    case VIRTIO_BLK_T_IN: {
      // Read operation: data goes to in_descs (except last one = status)
      if (in_descs.size() < 2) {
        status = VIRTIO_BLK_S_IOERR;
        break;
      }

      uint64_t sector = req.sector;
      for (size_t i = 0; i < in_descs.size() - 1; i++) {
        uint32_t len = in_descs[i].len;
        uint32_t num_sectors = len / VIRTIO_BLK_SECTOR_SIZE;

        std::vector<uint8_t> buf(len);
        if (!do_read(sector, buf.data(), num_sectors)) {
          status = VIRTIO_BLK_S_IOERR;
          break;
        }

        if (!queues[QUEUE_REQUEST].write_guest_mem(in_descs[i].addr, buf.data(), len)) {
          status = VIRTIO_BLK_S_IOERR;
          break;
        }

        sector += num_sectors;
        total_len += len;
      }
      break;
    }

    case VIRTIO_BLK_T_OUT: {
      // Write operation: data comes from out_descs (after header)
      if (readonly) {
        status = VIRTIO_BLK_S_IOERR;
        break;
      }

      uint64_t sector = req.sector;
      for (size_t i = 1; i < out_descs.size(); i++) {
        uint32_t len = out_descs[i].len;
        uint32_t num_sectors = len / VIRTIO_BLK_SECTOR_SIZE;

        std::vector<uint8_t> buf(len);
        if (!queues[QUEUE_REQUEST].read_guest_mem(out_descs[i].addr, buf.data(), len)) {
          status = VIRTIO_BLK_S_IOERR;
          break;
        }

        if (!do_write(sector, buf.data(), num_sectors)) {
          status = VIRTIO_BLK_S_IOERR;
          break;
        }

        sector += num_sectors;
      }
      break;
    }

    case VIRTIO_BLK_T_FLUSH:
      if (!do_flush()) {
        status = VIRTIO_BLK_S_IOERR;
      }
      break;

    case VIRTIO_BLK_T_GET_ID: {
      // Return device ID string
      if (in_descs.size() >= 2 && in_descs[0].len >= VIRTIO_BLK_ID_BYTES) {
        char id_buf[VIRTIO_BLK_ID_BYTES];
        memset(id_buf, 0, sizeof(id_buf));
        do_get_id(id_buf, sizeof(id_buf));
        queues[QUEUE_REQUEST].write_guest_mem(in_descs[0].addr, id_buf, VIRTIO_BLK_ID_BYTES);
        total_len = VIRTIO_BLK_ID_BYTES;
      }
      break;
    }

    default:
      status = VIRTIO_BLK_S_UNSUPP;
      break;
    }

    // Write status to last byte of last in descriptor
    if (!in_descs.empty()) {
      auto& status_desc = in_descs.back();
      // Status is the last byte
      queues[QUEUE_REQUEST].write_guest_mem(
        status_desc.addr + status_desc.len - 1, &status, 1);
      total_len++;
    }

    queues[QUEUE_REQUEST].put_used_buf(head, total_len);
  }

  // Raise interrupt to notify guest
  raise_interrupt(VIRTIO_INT_USED_RING);
}

bool virtio_block_t::do_read(uint64_t sector, uint8_t* buf, uint32_t num_sectors) {
  if (fd < 0)
    return false;

  off_t offset = sector * VIRTIO_BLK_SECTOR_SIZE;
  size_t len = num_sectors * VIRTIO_BLK_SECTOR_SIZE;

  ssize_t n = pread(fd, buf, len, offset);
  if (n < 0) {
    fprintf(stderr, "virtio-block: read error at sector %lu: %s\n",
            (unsigned long)sector, strerror(errno));
    return false;
  }

  // Pad with zeros if we read less than requested (past end of file)
  if ((size_t)n < len) {
    memset(buf + n, 0, len - n);
  }

  return true;
}

bool virtio_block_t::do_write(uint64_t sector, const uint8_t* buf, uint32_t num_sectors) {
  if (fd < 0 || readonly)
    return false;

  off_t offset = sector * VIRTIO_BLK_SECTOR_SIZE;
  size_t len = num_sectors * VIRTIO_BLK_SECTOR_SIZE;

  ssize_t n = pwrite(fd, buf, len, offset);
  if (n < 0 || (size_t)n != len) {
    fprintf(stderr, "virtio-block: write error at sector %lu: %s\n",
            (unsigned long)sector, strerror(errno));
    return false;
  }

  return true;
}

bool virtio_block_t::do_flush() {
  if (fd < 0)
    return false;

  if (fsync(fd) < 0) {
    fprintf(stderr, "virtio-block: flush error: %s\n", strerror(errno));
    return false;
  }

  return true;
}

bool virtio_block_t::do_get_id(char* buf, size_t len) {
  strncpy(buf, device_id.c_str(), len);
  if (len > 0)
    buf[len - 1] = '\0';
  return true;
}

// ============================================================================
// Device tree and factory functions
// ============================================================================

std::string virtio_block_generate_dts(const sim_t* sim UNUSED, const std::vector<std::string>& sargs UNUSED)
{
  // Only generate DTS if block device is enabled
  if (!is_virtio_block_enabled())
    return "";

  std::stringstream s;
  reg_t base = VIRTIO_BLOCK_BASE;
  reg_t sz = VIRTIO_BLOCK_SIZE;

  s << std::hex
    << "    virtio_blk@" << base << " {\n"
       "      compatible = \"virtio,mmio\";\n"
       "      interrupt-parent = <&PLIC>;\n"
       "      interrupts = <" << std::dec << VIRTIO_BLOCK_INTERRUPT_ID << ">;\n"
    << std::hex
    << "      reg = <0x" << (base >> 32) << " 0x" << (base & (uint32_t)-1)
    <<              " 0x" << (sz >> 32) << " 0x" << (sz & (uint32_t)-1) << ">;\n"
       "    };\n";
  return s.str();
}

int fdt_parse_virtio_block(const void *fdt, reg_t *addr, uint32_t *int_id,
                            const char *compatible)
{
  int nodeoffset, len, rc;
  const fdt32_t *reg_p;

  // Find the virtio block device node by checking address
  nodeoffset = fdt_node_offset_by_compatible(fdt, -1, compatible);
  while (nodeoffset >= 0) {
    reg_t node_addr = 0;
    rc = fdt_get_node_addr_size(fdt, nodeoffset, &node_addr, NULL, "reg");
    if (rc >= 0 && node_addr == VIRTIO_BLOCK_BASE) {
      // Found it
      if (addr) *addr = node_addr;

      reg_p = (fdt32_t *)fdt_getprop(fdt, nodeoffset, "interrupts", &len);
      if (int_id) {
        if (reg_p) {
          *int_id = fdt32_to_cpu(*reg_p);
        } else {
          *int_id = VIRTIO_BLOCK_INTERRUPT_ID;
        }
      }
      return 0;
    }
    nodeoffset = fdt_node_offset_by_compatible(fdt, nodeoffset, compatible);
  }

  return -ENODEV;
}

virtio_block_t* virtio_block_parse_from_fdt(const void* fdt, const sim_t* sim,
                                             reg_t* base,
                                             const std::vector<std::string>& sargs)
{
  // Check if block device is enabled
  if (!is_virtio_block_enabled())
    return nullptr;

  uint32_t int_id;

  // Check if this is a virtio mmio device at block device address
  if (fdt_parse_virtio_block(fdt, base, &int_id, "virtio,mmio") != 0)
    return nullptr;

  abstract_interrupt_controller_t* intctrl = sim->get_intctrl();
  simif_t* simif = const_cast<simif_t*>(static_cast<const simif_t*>(sim));

  const std::string& image_path = get_virtio_block_image_path();
  bool readonly = get_virtio_block_readonly();

  return new virtio_block_t(simif, intctrl, int_id, image_path, readonly);
}

REGISTER_BUILTIN_DEVICE(virtio_block, virtio_block_parse_from_fdt, virtio_block_generate_dts)
