// See LICENSE for license details.
#ifndef _RISCV_VIRTIO_H
#define _RISCV_VIRTIO_H

#include "abstract_device.h"
#include "abstract_interrupt_controller.h"
#include "simif.h"
#include <cstdint>
#include <cstring>
#include <vector>

// VirtIO MMIO register offsets (VirtIO 1.2 specification, section 4.2.2)
#define VIRTIO_MMIO_MAGIC_VALUE 0x000
#define VIRTIO_MMIO_VERSION 0x004
#define VIRTIO_MMIO_DEVICE_ID 0x008
#define VIRTIO_MMIO_VENDOR_ID 0x00c
#define VIRTIO_MMIO_DEVICE_FEATURES 0x010
#define VIRTIO_MMIO_DEVICE_FEATURES_SEL 0x014
#define VIRTIO_MMIO_DRIVER_FEATURES 0x020
#define VIRTIO_MMIO_DRIVER_FEATURES_SEL 0x024
#define VIRTIO_MMIO_QUEUE_SEL 0x030
#define VIRTIO_MMIO_QUEUE_NUM_MAX 0x034
#define VIRTIO_MMIO_QUEUE_NUM 0x038
#define VIRTIO_MMIO_QUEUE_READY 0x044
#define VIRTIO_MMIO_QUEUE_NOTIFY 0x050
#define VIRTIO_MMIO_INTERRUPT_STATUS 0x060
#define VIRTIO_MMIO_INTERRUPT_ACK 0x064
#define VIRTIO_MMIO_STATUS 0x070
#define VIRTIO_MMIO_QUEUE_DESC_LOW 0x080
#define VIRTIO_MMIO_QUEUE_DESC_HIGH 0x084
#define VIRTIO_MMIO_QUEUE_AVAIL_LOW 0x090
#define VIRTIO_MMIO_QUEUE_AVAIL_HIGH 0x094
#define VIRTIO_MMIO_QUEUE_USED_LOW 0x0a0
#define VIRTIO_MMIO_QUEUE_USED_HIGH 0x0a4
#define VIRTIO_MMIO_CONFIG_GENERATION 0x0fc
#define VIRTIO_MMIO_CONFIG 0x100

#define VIRTIO_MMIO_SIZE 0x200

// Magic value "virt" in little endian
#define VIRTIO_MMIO_MAGIC 0x74726976

// VirtIO version (2 = modern/non-legacy)
#define VIRTIO_MMIO_VERSION_2 2

// Vendor ID (using QEMU's vendor ID for compatibility)
#define VIRTIO_VENDOR_ID 0x554D4551

// Device status bits
#define VIRTIO_STATUS_ACKNOWLEDGE 0x01
#define VIRTIO_STATUS_DRIVER 0x02
#define VIRTIO_STATUS_DRIVER_OK 0x04
#define VIRTIO_STATUS_FEATURES_OK 0x08
#define VIRTIO_STATUS_DEVICE_NEEDS_RESET 0x40
#define VIRTIO_STATUS_FAILED 0x80

// Interrupt status bits
#define VIRTIO_INT_USED_RING 0x01
#define VIRTIO_INT_CONFIG_CHANGE 0x02

// Virtqueue descriptor flags
#define VRING_DESC_F_NEXT 0x01
#define VRING_DESC_F_WRITE 0x02
#define VRING_DESC_F_INDIRECT 0x04

// Available ring flags (set by driver to control notifications)
#define VRING_AVAIL_F_NO_INTERRUPT 0x01

// Common feature bits
#define VIRTIO_F_INDIRECT_DESC (1ULL << 28)
#define VIRTIO_F_EVENT_IDX (1ULL << 29)
#define VIRTIO_F_VERSION_1 (1ULL << 32)
#define VIRTIO_F_RING_PACKED (1ULL << 34)

// Virtqueue descriptor structure
struct vring_desc {
  uint64_t addr;  // Guest physical address
  uint32_t len;   // Length of buffer
  uint16_t flags; // VRING_DESC_F_* flags
  uint16_t next;  // Index of next descriptor if chained
};

// Available ring structure (header only, ring follows)
struct vring_avail {
  uint16_t flags;
  uint16_t idx;
  // uint16_t ring[] follows
};

// Used ring element
struct vring_used_elem {
  uint32_t id;  // Index of descriptor chain head
  uint32_t len; // Total bytes written to device
};

// Used ring structure (header only, ring follows)
struct vring_used {
  uint16_t flags;
  uint16_t idx;
  // vring_used_elem ring[] follows
};

// Virtqueue class for managing a single virtqueue
class virtqueue_t {
public:
  virtqueue_t(simif_t *sim, uint32_t max_queue_size);

  void reset();
  void set_num(uint32_t num);
  void set_desc_addr(uint64_t addr);
  void set_avail_addr(uint64_t addr);
  void set_used_addr(uint64_t addr);
  void set_ready(bool ready);

  uint32_t get_num() const { return num; }
  uint32_t get_max_num() const { return max_num; }
  bool is_ready() const { return ready; }

  // Check if there are pending descriptors to process
  bool has_pending();

  // Check if the driver wants to be notified
  // Implements both VRING_AVAIL_F_NO_INTERRUPT and VIRTIO_F_EVENT_IDX
  bool should_notify();

  // Set whether VIRTIO_F_EVENT_IDX was negotiated
  void set_event_idx(bool enabled) { event_idx_enabled = enabled; }

  // Get next available descriptor chain
  // Returns head descriptor index, or -1 if none available
  // out_descs: device-readable descriptors (guest writes, device reads)
  // in_descs: device-writable descriptors (device writes, guest reads)
  int get_avail_buf(std::vector<vring_desc> &out_descs,
                    std::vector<vring_desc> &in_descs);

  // Return processed buffer to guest
  void put_used_buf(uint16_t head, uint32_t len);

  // Read from guest memory
  bool read_guest_mem(uint64_t addr, void *buf, size_t len);

  // Write to guest memory
  bool write_guest_mem(uint64_t addr, const void *buf, size_t len);

private:
  simif_t *sim;
  uint32_t max_num;
  uint32_t num;
  uint64_t desc_addr;
  uint64_t avail_addr;
  uint64_t used_addr;
  uint16_t last_avail_idx;
  uint16_t last_used_idx;      // Track used idx for EVENT_IDX notification logic
  uint16_t old_used_idx;       // Previous used idx for EVENT_IDX comparison
  bool ready;
  bool event_idx_enabled;      // True if VIRTIO_F_EVENT_IDX was negotiated
};

// VirtIO base class implementing MMIO transport
class virtio_base_t : public abstract_device_t {
public:
  virtio_base_t(simif_t *sim, abstract_interrupt_controller_t *intctrl,
                uint32_t interrupt_id, uint32_t num_queues,
                uint32_t queue_size);
  virtual ~virtio_base_t();

  // abstract_device_t interface
  bool load(reg_t addr, size_t len, uint8_t *bytes) override;
  bool store(reg_t addr, size_t len, const uint8_t *bytes) override;
  reg_t size() override { return VIRTIO_MMIO_SIZE; }
  void tick(reg_t rtc_ticks) override;

protected:
  // Device-specific overrides
  virtual uint32_t get_device_id() const = 0;
  virtual uint64_t get_device_features() const = 0;
  virtual void handle_driver_features(uint64_t features) {}
  virtual void handle_queue_notify(uint32_t queue_idx) = 0;
  virtual bool read_config(reg_t offset, size_t len, uint8_t *bytes) = 0;
  virtual bool write_config(reg_t offset, size_t len, const uint8_t *bytes) = 0;
  virtual uint32_t get_config_size() const = 0;
  virtual void device_reset() {}

  // Raise interrupt (reason: VIRTIO_INT_USED_RING or VIRTIO_INT_CONFIG_CHANGE)
  void raise_interrupt(uint32_t reason);
  void update_interrupt();

  // Check if interrupt is already pending (for coalescing)
  bool is_interrupt_pending() const;

  simif_t *sim;
  abstract_interrupt_controller_t *intctrl;
  uint32_t interrupt_id;

  std::vector<virtqueue_t> queues;

private:
  // MMIO registers
  uint32_t device_features_sel;
  uint32_t driver_features_lo;
  uint32_t driver_features_hi;
  uint32_t driver_features_sel;
  uint32_t queue_sel;
  uint32_t interrupt_status;
  uint32_t status;
  uint32_t config_generation;

  // Temporary storage for 64-bit queue address writes
  uint32_t desc_addr_lo;
  uint32_t avail_addr_lo;
  uint32_t used_addr_lo;
};

#endif // _RISCV_VIRTIO_H
