// See LICENSE for license details.
#include "virtio.h"
#include "mmu.h"
#include <cassert>
#include <cstring>

// Helper functions for little-endian register access
static uint32_t read_le32(const uint8_t *bytes) {
  return bytes[0] | (bytes[1] << 8) | (bytes[2] << 16) | (bytes[3] << 24);
}

static void write_le32(uint8_t *bytes, uint32_t val) {
  bytes[0] = val & 0xff;
  bytes[1] = (val >> 8) & 0xff;
  bytes[2] = (val >> 16) & 0xff;
  bytes[3] = (val >> 24) & 0xff;
}

// ============================================================================
// virtqueue_t implementation
// ============================================================================

virtqueue_t::virtqueue_t(simif_t *sim, uint32_t max_queue_size)
    : sim(sim), max_num(max_queue_size), num(0), desc_addr(0), avail_addr(0),
      used_addr(0), last_avail_idx(0), last_used_idx(0), old_used_idx(0),
      ready(false), event_idx_enabled(false) {}

void virtqueue_t::reset() {
  num = 0;
  desc_addr = 0;
  avail_addr = 0;
  used_addr = 0;
  last_avail_idx = 0;
  last_used_idx = 0;
  old_used_idx = 0;
  ready = false;
  event_idx_enabled = false;
}

void virtqueue_t::set_num(uint32_t n) {
  if (n <= max_num && (n & (n - 1)) == 0) { // Must be power of 2
    num = n;
  }
}

void virtqueue_t::set_desc_addr(uint64_t addr) { desc_addr = addr; }

void virtqueue_t::set_avail_addr(uint64_t addr) { avail_addr = addr; }

void virtqueue_t::set_used_addr(uint64_t addr) { used_addr = addr; }

void virtqueue_t::set_ready(bool r) {
  ready = r;
  if (ready) {
    last_avail_idx = 0;
  }
}

bool virtqueue_t::read_guest_mem(uint64_t addr, void *buf, size_t len) {
  return sim->mmio_load(addr, len, (uint8_t *)buf);
}

bool virtqueue_t::write_guest_mem(uint64_t addr, const void *buf, size_t len) {
  return sim->mmio_store(addr, len, (const uint8_t *)buf);
}

bool virtqueue_t::has_pending() {
  if (!ready || num == 0)
    return false;

  uint16_t avail_idx;
  if (!read_guest_mem(avail_addr + offsetof(vring_avail, idx), &avail_idx,
                      sizeof(avail_idx)))
    return false;

  return last_avail_idx != avail_idx;
}

// Helper function from VirtIO spec: check if we need to notify based on EVENT_IDX
// Returns true if event_idx is in the range (old_idx, new_idx]
static inline bool vring_need_event(uint16_t event_idx, uint16_t new_idx,
                                    uint16_t old_idx) {
  return (uint16_t)(new_idx - event_idx - 1) < (uint16_t)(new_idx - old_idx);
}

bool virtqueue_t::should_notify() {
  static bool debug_virtio = getenv("DEBUG_VIRTIO") != nullptr;
  static uint64_t check_count = 0;
  static uint64_t suppress_count = 0;
  static uint64_t event_idx_suppress_count = 0;
  check_count++;

  if (!ready || num == 0 || avail_addr == 0)
    return true; // Default to notifying if not properly configured

  // If EVENT_IDX is negotiated, use used_event threshold instead of flags
  if (event_idx_enabled) {
    // Read used_event from end of avail ring: avail_addr + 4 + num * 2
    // (4 bytes for flags+idx, then num uint16_t ring entries)
    uint64_t used_event_addr = avail_addr + sizeof(vring_avail) +
                                num * sizeof(uint16_t);
    uint16_t used_event;
    if (!read_guest_mem(used_event_addr, &used_event, sizeof(used_event)))
      return true; // Can't read - be safe and notify

    // Check if we crossed the used_event threshold
    bool should = vring_need_event(used_event, last_used_idx, old_used_idx);
    old_used_idx = last_used_idx;

    if (!should) {
      event_idx_suppress_count++;
      if (debug_virtio && event_idx_suppress_count % 1000 == 1) {
        fprintf(stderr,
                "virtqueue: EVENT_IDX suppressed, used_event=%u, "
                "last_used_idx=%u, suppressed=%lu/%lu\n",
                used_event, last_used_idx, event_idx_suppress_count,
                check_count);
      }
    } else if (debug_virtio && check_count % 10000 == 1) {
      fprintf(stderr,
              "virtqueue: EVENT_IDX notify, used_event=%u, last_used_idx=%u\n",
              used_event, last_used_idx);
    }
    return should;
  }

  // Fall back to VRING_AVAIL_F_NO_INTERRUPT check
  uint16_t avail_flags;
  if (!read_guest_mem(avail_addr + offsetof(vring_avail, flags), &avail_flags,
                      sizeof(avail_flags)))
    return true; // Can't read - be safe and notify

  // If VRING_AVAIL_F_NO_INTERRUPT is set, driver doesn't want notifications
  bool should = !(avail_flags & VRING_AVAIL_F_NO_INTERRUPT);
  if (!should) {
    suppress_count++;
    if (debug_virtio && suppress_count % 1000 == 1) {
      fprintf(stderr,
              "virtqueue: should_notify=false, avail_flags=0x%x, "
              "suppressed=%lu/%lu\n",
              avail_flags, suppress_count, check_count);
    }
  }
  return should;
}

int virtqueue_t::get_avail_buf(std::vector<vring_desc> &out_descs,
                               std::vector<vring_desc> &in_descs) {
  out_descs.clear();
  in_descs.clear();

  if (!ready || num == 0)
    return -1;

  // Read current available index
  uint16_t avail_idx;
  if (!read_guest_mem(avail_addr + offsetof(vring_avail, idx), &avail_idx,
                      sizeof(avail_idx)))
    return -1;

  // Check if there are new buffers
  if (last_avail_idx == avail_idx)
    return -1;

  // Get descriptor head index from available ring
  uint16_t ring_offset = (last_avail_idx % num) * sizeof(uint16_t);
  uint16_t head;
  if (!read_guest_mem(avail_addr + sizeof(vring_avail) + ring_offset, &head,
                      sizeof(head)))
    return -1;

  // Walk descriptor chain
  uint16_t desc_idx = head;
  int chain_count = 0;
  const int max_chain = 1024; // Prevent infinite loops

  while (chain_count < max_chain) {
    vring_desc desc;
    if (!read_guest_mem(desc_addr + desc_idx * sizeof(vring_desc), &desc,
                        sizeof(desc)))
      return -1;

    // Handle indirect descriptor
    if (desc.flags & VRING_DESC_F_INDIRECT) {
      // desc.addr points to a table of indirect descriptors
      // desc.len is the total size of the indirect table
      uint64_t indirect_addr = desc.addr;
      uint32_t indirect_count = desc.len / sizeof(vring_desc);

      // Walk the indirect descriptor table
      for (uint32_t i = 0; i < indirect_count && chain_count < max_chain; i++) {
        vring_desc indirect_desc;
        if (!read_guest_mem(indirect_addr + i * sizeof(vring_desc),
                            &indirect_desc, sizeof(indirect_desc)))
          return -1;

        // Add to appropriate list based on WRITE flag
        if (indirect_desc.flags & VRING_DESC_F_WRITE) {
          in_descs.push_back(indirect_desc);
        } else {
          out_descs.push_back(indirect_desc);
        }

        chain_count++;

        // Check if there's a next descriptor in the indirect table
        if (!(indirect_desc.flags & VRING_DESC_F_NEXT))
          break;
      }

      // After processing indirect, check if there are more descriptors in main
      // chain
      if (!(desc.flags & VRING_DESC_F_NEXT))
        break;

      desc_idx = desc.next;
      chain_count++;
      continue;
    }

    // Separate device-readable (out) and device-writable (in) descriptors
    if (desc.flags & VRING_DESC_F_WRITE) {
      in_descs.push_back(desc);
    } else {
      out_descs.push_back(desc);
    }

    if (!(desc.flags & VRING_DESC_F_NEXT))
      break;

    desc_idx = desc.next;
    chain_count++;
  }

  last_avail_idx++;
  return head;
}

void virtqueue_t::put_used_buf(uint16_t head, uint32_t len) {
  if (!ready || num == 0)
    return;

  // Read current used index
  uint16_t used_idx;
  if (!read_guest_mem(used_addr + offsetof(vring_used, idx), &used_idx,
                      sizeof(used_idx)))
    return;

  // Write used element
  vring_used_elem elem;
  elem.id = head;
  elem.len = len;

  uint16_t ring_offset = (used_idx % num) * sizeof(vring_used_elem);
  write_guest_mem(used_addr + sizeof(vring_used) + ring_offset, &elem,
                  sizeof(elem));

  // Update used index and track for EVENT_IDX notification logic
  used_idx++;
  write_guest_mem(used_addr + offsetof(vring_used, idx), &used_idx,
                  sizeof(used_idx));
  last_used_idx = used_idx;
}

// ============================================================================
// virtio_base_t implementation
// ============================================================================

virtio_base_t::virtio_base_t(simif_t *sim,
                             abstract_interrupt_controller_t *intctrl,
                             uint32_t interrupt_id, uint32_t num_queues,
                             uint32_t queue_size)
    : sim(sim), intctrl(intctrl), interrupt_id(interrupt_id),
      device_features_sel(0), driver_features_lo(0), driver_features_hi(0),
      driver_features_sel(0), queue_sel(0), interrupt_status(0), status(0),
      config_generation(0), desc_addr_lo(0), avail_addr_lo(0), used_addr_lo(0) {
  for (uint32_t i = 0; i < num_queues; i++) {
    queues.emplace_back(sim, queue_size);
  }
}

virtio_base_t::~virtio_base_t() {}

void virtio_base_t::raise_interrupt(uint32_t reason) {
  static bool debug_virtio = getenv("DEBUG_VIRTIO") != nullptr;
  interrupt_status |= reason;
  if (debug_virtio) {
    fprintf(
        stderr,
        "virtio: device_id=%d raise_interrupt reason=%u status=%u int_id=%u\n",
        get_device_id(), reason, interrupt_status, interrupt_id);
  }
  update_interrupt();
}

void virtio_base_t::update_interrupt() {
  if (interrupt_status) {
    intctrl->set_interrupt_level(interrupt_id, 1);
  } else {
    intctrl->set_interrupt_level(interrupt_id, 0);
  }
}

bool virtio_base_t::is_interrupt_pending() const {
  return interrupt_status != 0;
}

bool virtio_base_t::load(reg_t addr, size_t len, uint8_t *bytes) {
  static bool debug_virtio = getenv("DEBUG_VIRTIO") != nullptr;

  // Config space can be read with any size (1, 2, or 4 bytes)
  if (addr >= VIRTIO_MMIO_CONFIG &&
      addr < VIRTIO_MMIO_CONFIG + get_config_size()) {
    if (debug_virtio)
      fprintf(
          stderr,
          "virtio: device_id=%d config read addr=0x%lx offset=%lu len=%zu\n",
          get_device_id(), (unsigned long)addr,
          (unsigned long)(addr - VIRTIO_MMIO_CONFIG), len);
    if (!read_config(addr - VIRTIO_MMIO_CONFIG, len, bytes))
      memset(bytes, 0, len);
    return true;
  }

  if (len != 4) {
    // VirtIO MMIO standard registers use 32-bit access
    memset(bytes, 0, len);
    return true;
  }

  uint32_t val = 0;

  switch (addr) {
  case VIRTIO_MMIO_MAGIC_VALUE:
    val = VIRTIO_MMIO_MAGIC;
    if (debug_virtio)
      fprintf(stderr, "virtio: device_id=%d magic read\n", get_device_id());
    break;

  case VIRTIO_MMIO_VERSION:
    val = VIRTIO_MMIO_VERSION_2;
    if (debug_virtio)
      fprintf(stderr, "virtio: device_id=%d version read -> %d\n",
              get_device_id(), val);
    break;

  case VIRTIO_MMIO_DEVICE_ID:
    val = get_device_id();
    if (debug_virtio)
      fprintf(stderr, "virtio: device_id read -> %d\n", val);
    break;

  case VIRTIO_MMIO_VENDOR_ID:
    val = VIRTIO_VENDOR_ID;
    break;

  case VIRTIO_MMIO_DEVICE_FEATURES: {
    uint64_t features = get_device_features();
    if (device_features_sel == 0)
      val = features & 0xffffffff;
    else if (device_features_sel == 1)
      val = (features >> 32) & 0xffffffff;
    else
      val = 0;
  } break;

  case VIRTIO_MMIO_QUEUE_NUM_MAX:
    if (queue_sel < queues.size())
      val = queues[queue_sel].get_max_num();
    else
      val = 0;
    break;

  case VIRTIO_MMIO_QUEUE_READY:
    if (queue_sel < queues.size())
      val = queues[queue_sel].is_ready() ? 1 : 0;
    else
      val = 0;
    break;

  case VIRTIO_MMIO_INTERRUPT_STATUS:
    val = interrupt_status;
    break;

  case VIRTIO_MMIO_STATUS:
    val = status;
    break;

  case VIRTIO_MMIO_CONFIG_GENERATION:
    val = config_generation;
    break;

  default:
    // Config space is handled at the top of load()
    if (debug_virtio)
      fprintf(stderr, "virtio: device_id=%d unknown read addr=0x%lx\n",
              get_device_id(), (unsigned long)addr);
    val = 0;
    break;
  }

  write_le32(bytes, val);
  return true;
}

bool virtio_base_t::store(reg_t addr, size_t len, const uint8_t *bytes) {
  static bool debug_virtio = getenv("DEBUG_VIRTIO") != nullptr;

  if (len != 4) {
    return true; // Ignore non-32-bit writes
  }

  uint32_t val = read_le32(bytes);

  switch (addr) {
  case VIRTIO_MMIO_DEVICE_FEATURES_SEL:
    device_features_sel = val;
    break;

  case VIRTIO_MMIO_DRIVER_FEATURES:
    if (driver_features_sel == 0)
      driver_features_lo = val;
    else if (driver_features_sel == 1)
      driver_features_hi = val;
    break;

  case VIRTIO_MMIO_DRIVER_FEATURES_SEL:
    driver_features_sel = val;
    break;

  case VIRTIO_MMIO_QUEUE_SEL:
    queue_sel = val;
    break;

  case VIRTIO_MMIO_QUEUE_NUM:
    if (queue_sel < queues.size())
      queues[queue_sel].set_num(val);
    break;

  case VIRTIO_MMIO_QUEUE_READY:
    if (queue_sel < queues.size()) {
      if (debug_virtio)
        fprintf(stderr, "virtio: device_id=%d queue[%d] ready=%d this=%p\n",
                get_device_id(), queue_sel, val, (void *)this);
      queues[queue_sel].set_ready(val != 0);
    }
    break;

  case VIRTIO_MMIO_QUEUE_NOTIFY:
    if (debug_virtio)
      fprintf(stderr, "virtio: device_id=%d QUEUE_NOTIFY val=%d\n",
              get_device_id(), val);
    if (val < queues.size())
      handle_queue_notify(val);
    break;

  case VIRTIO_MMIO_INTERRUPT_ACK:
    if (debug_virtio)
      fprintf(stderr,
              "virtio: device_id=%d INTERRUPT_ACK val=%u status=%u->%u\n",
              get_device_id(), val, interrupt_status, interrupt_status & ~val);
    interrupt_status &= ~val;
    update_interrupt();
    break;

  case VIRTIO_MMIO_STATUS:
    if (debug_virtio)
      fprintf(stderr, "virtio: device_id=%d status write %02x -> %02x\n",
              get_device_id(), status, val);
    if (val == 0) {
      // Device reset
      status = 0;
      interrupt_status = 0;
      device_features_sel = 0;
      driver_features_lo = 0;
      driver_features_hi = 0;
      driver_features_sel = 0;
      queue_sel = 0;
      for (auto &q : queues)
        q.reset();
      device_reset();
      update_interrupt();
    } else {
      status = val;
      if ((status & VIRTIO_STATUS_FEATURES_OK) &&
          !(status & VIRTIO_STATUS_DRIVER_OK)) {
        // Features negotiation complete
        uint64_t features =
            ((uint64_t)driver_features_hi << 32) | driver_features_lo;
        if (debug_virtio)
          fprintf(stderr, "virtio: device_id=%d features negotiated %016llx\n",
                  get_device_id(), (unsigned long long)features);
        handle_driver_features(features);
      }
    }
    break;

  case VIRTIO_MMIO_QUEUE_DESC_LOW:
    if (queue_sel < queues.size()) {
      // Store low 32 bits - will be combined with high bits when set_desc_addr
      // is called
      desc_addr_lo = val;
    }
    break;

  case VIRTIO_MMIO_QUEUE_DESC_HIGH:
    if (queue_sel < queues.size()) {
      // Combine with previously set low bits
      uint64_t addr = ((uint64_t)val << 32) | desc_addr_lo;
      queues[queue_sel].set_desc_addr(addr);
      if (debug_virtio)
        fprintf(stderr, "virtio: device_id=%d queue[%d] desc_addr=0x%016llx\n",
                get_device_id(), queue_sel, (unsigned long long)addr);
    }
    break;

  case VIRTIO_MMIO_QUEUE_AVAIL_LOW:
    if (queue_sel < queues.size()) {
      avail_addr_lo = val;
    }
    break;

  case VIRTIO_MMIO_QUEUE_AVAIL_HIGH:
    if (queue_sel < queues.size()) {
      uint64_t addr = ((uint64_t)val << 32) | avail_addr_lo;
      queues[queue_sel].set_avail_addr(addr);
      if (debug_virtio)
        fprintf(stderr, "virtio: device_id=%d queue[%d] avail_addr=0x%016llx\n",
                get_device_id(), queue_sel, (unsigned long long)addr);
    }
    break;

  case VIRTIO_MMIO_QUEUE_USED_LOW:
    if (queue_sel < queues.size()) {
      used_addr_lo = val;
    }
    break;

  case VIRTIO_MMIO_QUEUE_USED_HIGH:
    if (queue_sel < queues.size()) {
      uint64_t addr = ((uint64_t)val << 32) | used_addr_lo;
      queues[queue_sel].set_used_addr(addr);
      if (debug_virtio)
        fprintf(stderr, "virtio: device_id=%d queue[%d] used_addr=0x%016llx\n",
                get_device_id(), queue_sel, (unsigned long long)addr);
    }
    break;

  default:
    if (addr >= VIRTIO_MMIO_CONFIG &&
        addr < VIRTIO_MMIO_CONFIG + get_config_size()) {
      write_config(addr - VIRTIO_MMIO_CONFIG, len, bytes);
    }
    break;
  }

  return true;
}

void virtio_base_t::tick(reg_t rtc_ticks) {
  // Default implementation does nothing
  // Subclasses can override to poll for external events
}
