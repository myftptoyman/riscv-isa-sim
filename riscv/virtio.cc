// See LICENSE for license details.
#include "virtio.h"
#include "mmu.h"
#include <cassert>
#include <cstring>

// Helper functions for little-endian register access
static uint32_t read_le32(const uint8_t* bytes) {
  return bytes[0] | (bytes[1] << 8) | (bytes[2] << 16) | (bytes[3] << 24);
}

static void write_le32(uint8_t* bytes, uint32_t val) {
  bytes[0] = val & 0xff;
  bytes[1] = (val >> 8) & 0xff;
  bytes[2] = (val >> 16) & 0xff;
  bytes[3] = (val >> 24) & 0xff;
}

// ============================================================================
// virtqueue_t implementation
// ============================================================================

virtqueue_t::virtqueue_t(simif_t* sim, uint32_t max_queue_size)
  : sim(sim), max_num(max_queue_size), num(0),
    desc_addr(0), avail_addr(0), used_addr(0),
    last_avail_idx(0), ready(false)
{
}

void virtqueue_t::reset() {
  num = 0;
  desc_addr = 0;
  avail_addr = 0;
  used_addr = 0;
  last_avail_idx = 0;
  ready = false;
}

void virtqueue_t::set_num(uint32_t n) {
  if (n <= max_num && (n & (n - 1)) == 0) {  // Must be power of 2
    num = n;
  }
}

void virtqueue_t::set_desc_addr(uint64_t addr) {
  desc_addr = addr;
}

void virtqueue_t::set_avail_addr(uint64_t addr) {
  avail_addr = addr;
}

void virtqueue_t::set_used_addr(uint64_t addr) {
  used_addr = addr;
}

void virtqueue_t::set_ready(bool r) {
  ready = r;
  if (ready) {
    last_avail_idx = 0;
  }
}

bool virtqueue_t::read_guest_mem(uint64_t addr, void* buf, size_t len) {
  return sim->mmio_load(addr, len, (uint8_t*)buf);
}

bool virtqueue_t::write_guest_mem(uint64_t addr, const void* buf, size_t len) {
  return sim->mmio_store(addr, len, (const uint8_t*)buf);
}

bool virtqueue_t::has_pending() {
  if (!ready || num == 0) return false;

  uint16_t avail_idx;
  if (!read_guest_mem(avail_addr + offsetof(vring_avail, idx), &avail_idx, sizeof(avail_idx)))
    return false;

  return last_avail_idx != avail_idx;
}

int virtqueue_t::get_avail_buf(std::vector<vring_desc>& out_descs, std::vector<vring_desc>& in_descs) {
  out_descs.clear();
  in_descs.clear();

  if (!ready || num == 0) return -1;

  // Read current available index
  uint16_t avail_idx;
  if (!read_guest_mem(avail_addr + offsetof(vring_avail, idx), &avail_idx, sizeof(avail_idx)))
    return -1;

  // Check if there are new buffers
  if (last_avail_idx == avail_idx)
    return -1;

  // Get descriptor head index from available ring
  uint16_t ring_offset = (last_avail_idx % num) * sizeof(uint16_t);
  uint16_t head;
  if (!read_guest_mem(avail_addr + sizeof(vring_avail) + ring_offset, &head, sizeof(head)))
    return -1;

  // Walk descriptor chain
  uint16_t desc_idx = head;
  int chain_count = 0;
  const int max_chain = 256;  // Prevent infinite loops

  while (chain_count < max_chain) {
    vring_desc desc;
    if (!read_guest_mem(desc_addr + desc_idx * sizeof(vring_desc), &desc, sizeof(desc)))
      return -1;

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
  if (!ready || num == 0) return;

  // Read current used index
  uint16_t used_idx;
  if (!read_guest_mem(used_addr + offsetof(vring_used, idx), &used_idx, sizeof(used_idx)))
    return;

  // Write used element
  vring_used_elem elem;
  elem.id = head;
  elem.len = len;

  uint16_t ring_offset = (used_idx % num) * sizeof(vring_used_elem);
  write_guest_mem(used_addr + sizeof(vring_used) + ring_offset, &elem, sizeof(elem));

  // Update used index
  used_idx++;
  write_guest_mem(used_addr + offsetof(vring_used, idx), &used_idx, sizeof(used_idx));
}

// ============================================================================
// virtio_base_t implementation
// ============================================================================

virtio_base_t::virtio_base_t(simif_t* sim,
                             abstract_interrupt_controller_t* intctrl,
                             uint32_t interrupt_id,
                             uint32_t num_queues,
                             uint32_t queue_size)
  : sim(sim), intctrl(intctrl), interrupt_id(interrupt_id),
    device_features_sel(0), driver_features_lo(0), driver_features_hi(0),
    driver_features_sel(0), queue_sel(0), interrupt_status(0),
    status(0), config_generation(0)
{
  for (uint32_t i = 0; i < num_queues; i++) {
    queues.emplace_back(sim, queue_size);
  }
}

virtio_base_t::~virtio_base_t() {
}

void virtio_base_t::raise_interrupt(uint32_t reason) {
  interrupt_status |= reason;
  update_interrupt();
}

void virtio_base_t::update_interrupt() {
  if (interrupt_status != 0) {
    intctrl->set_interrupt_level(interrupt_id, 1);
  } else {
    intctrl->set_interrupt_level(interrupt_id, 0);
  }
}

bool virtio_base_t::load(reg_t addr, size_t len, uint8_t* bytes) {
  if (len != 4) {
    // VirtIO MMIO uses 32-bit registers
    memset(bytes, 0, len);
    return true;
  }

  uint32_t val = 0;

  switch (addr) {
    case VIRTIO_MMIO_MAGIC_VALUE:
      val = VIRTIO_MMIO_MAGIC;
      break;

    case VIRTIO_MMIO_VERSION:
      val = VIRTIO_MMIO_VERSION_2;
      break;

    case VIRTIO_MMIO_DEVICE_ID:
      val = get_device_id();
      break;

    case VIRTIO_MMIO_VENDOR_ID:
      val = VIRTIO_VENDOR_ID;
      break;

    case VIRTIO_MMIO_DEVICE_FEATURES:
      {
        uint64_t features = get_device_features();
        if (device_features_sel == 0)
          val = features & 0xffffffff;
        else if (device_features_sel == 1)
          val = (features >> 32) & 0xffffffff;
        else
          val = 0;
      }
      break;

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
      if (addr >= VIRTIO_MMIO_CONFIG && addr < VIRTIO_MMIO_CONFIG + get_config_size()) {
        if (!read_config(addr - VIRTIO_MMIO_CONFIG, len, bytes))
          memset(bytes, 0, len);
        return true;
      }
      val = 0;
      break;
  }

  write_le32(bytes, val);
  return true;
}

bool virtio_base_t::store(reg_t addr, size_t len, const uint8_t* bytes) {
  if (len != 4) {
    return true;  // Ignore non-32-bit writes
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
      if (queue_sel < queues.size())
        queues[queue_sel].set_ready(val != 0);
      break;

    case VIRTIO_MMIO_QUEUE_NOTIFY:
      if (val < queues.size())
        handle_queue_notify(val);
      break;

    case VIRTIO_MMIO_INTERRUPT_ACK:
      interrupt_status &= ~val;
      update_interrupt();
      break;

    case VIRTIO_MMIO_STATUS:
      if (val == 0) {
        // Device reset
        status = 0;
        interrupt_status = 0;
        device_features_sel = 0;
        driver_features_lo = 0;
        driver_features_hi = 0;
        driver_features_sel = 0;
        queue_sel = 0;
        for (auto& q : queues)
          q.reset();
        device_reset();
        update_interrupt();
      } else {
        status = val;
        if ((status & VIRTIO_STATUS_FEATURES_OK) && !(status & VIRTIO_STATUS_DRIVER_OK)) {
          // Features negotiation complete
          uint64_t features = ((uint64_t)driver_features_hi << 32) | driver_features_lo;
          handle_driver_features(features);
        }
      }
      break;

    case VIRTIO_MMIO_QUEUE_DESC_LOW:
      if (queue_sel < queues.size()) {
        uint64_t addr = queues[queue_sel].get_num();  // Dummy read to preserve high bits
        (void)addr;
        // For simplicity, just set low 32 bits
        queues[queue_sel].set_desc_addr((queues[queue_sel].get_num() ? 0 : 0) | val);
      }
      break;

    case VIRTIO_MMIO_QUEUE_DESC_HIGH:
      if (queue_sel < queues.size()) {
        // Combine with previously set low bits - simplified approach
        // In reality we'd need to track partial writes
      }
      break;

    case VIRTIO_MMIO_QUEUE_AVAIL_LOW:
      if (queue_sel < queues.size())
        queues[queue_sel].set_avail_addr(val);
      break;

    case VIRTIO_MMIO_QUEUE_AVAIL_HIGH:
      // Handle 64-bit address (for rv64)
      break;

    case VIRTIO_MMIO_QUEUE_USED_LOW:
      if (queue_sel < queues.size())
        queues[queue_sel].set_used_addr(val);
      break;

    case VIRTIO_MMIO_QUEUE_USED_HIGH:
      // Handle 64-bit address (for rv64)
      break;

    default:
      if (addr >= VIRTIO_MMIO_CONFIG && addr < VIRTIO_MMIO_CONFIG + get_config_size()) {
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
