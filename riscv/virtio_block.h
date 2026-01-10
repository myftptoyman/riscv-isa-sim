// See LICENSE for license details.
#ifndef _RISCV_VIRTIO_BLOCK_H
#define _RISCV_VIRTIO_BLOCK_H

#include "virtio.h"
#include <string>
#include <vector>

// VirtIO Block device ID (VirtIO 1.2 specification)
#define VIRTIO_ID_BLOCK         0x02

// VirtIO Block feature bits (VirtIO 1.2 spec section 5.2.3)
#define VIRTIO_BLK_F_SIZE_MAX   (1ULL << 1)   // Max size of any single segment
#define VIRTIO_BLK_F_SEG_MAX    (1ULL << 2)   // Max number of segments in a request
#define VIRTIO_BLK_F_GEOMETRY   (1ULL << 4)   // Disk geometry is available
#define VIRTIO_BLK_F_RO         (1ULL << 5)   // Device is read-only
#define VIRTIO_BLK_F_BLK_SIZE   (1ULL << 6)   // Block size is available
#define VIRTIO_BLK_F_FLUSH      (1ULL << 9)   // Cache flush command support
#define VIRTIO_BLK_F_TOPOLOGY   (1ULL << 10)  // Topology information is available
#define VIRTIO_BLK_F_CONFIG_WCE (1ULL << 11)  // Writeback mode available
#define VIRTIO_BLK_F_DISCARD    (1ULL << 13)  // DISCARD is supported
#define VIRTIO_BLK_F_WRITE_ZEROES (1ULL << 14) // WRITE ZEROES is supported

// Block request types (VirtIO 1.2 spec section 5.2.6)
#define VIRTIO_BLK_T_IN         0   // Read from device
#define VIRTIO_BLK_T_OUT        1   // Write to device
#define VIRTIO_BLK_T_FLUSH      4   // Flush (cache writeback)
#define VIRTIO_BLK_T_GET_ID     8   // Get device ID string
#define VIRTIO_BLK_T_DISCARD    11  // Discard sectors
#define VIRTIO_BLK_T_WRITE_ZEROES 13 // Write zeroes

// Block request status values
#define VIRTIO_BLK_S_OK         0   // Success
#define VIRTIO_BLK_S_IOERR      1   // Device or driver error
#define VIRTIO_BLK_S_UNSUPP     2   // Request unsupported

// Default sector size
#define VIRTIO_BLK_SECTOR_SIZE  512

// Default queue size
#define VIRTIO_BLK_QUEUE_SIZE   128

// Device ID string max length
#define VIRTIO_BLK_ID_BYTES     20

// Block device configuration space (VirtIO 1.2 section 5.2.4)
struct virtio_blk_config {
  uint64_t capacity;        // Total sectors (512 bytes each)
  uint32_t size_max;        // Max size of any single segment (if VIRTIO_BLK_F_SIZE_MAX)
  uint32_t seg_max;         // Max number of segments in a request (if VIRTIO_BLK_F_SEG_MAX)
  struct {
    uint16_t cylinders;
    uint8_t heads;
    uint8_t sectors;
  } geometry;               // Disk geometry (if VIRTIO_BLK_F_GEOMETRY)
  uint32_t blk_size;        // Block size in bytes (if VIRTIO_BLK_F_BLK_SIZE)
  // Additional fields for topology, discard, etc. omitted for simplicity
};

// Block I/O request header (at start of each request)
struct virtio_blk_req {
  uint32_t type;            // VIRTIO_BLK_T_*
  uint32_t reserved;
  uint64_t sector;          // Starting sector for read/write
};

class virtio_block_t : public virtio_base_t {
public:
  // Queue index (block device has single request queue)
  static const uint32_t QUEUE_REQUEST = 0;
  static const uint32_t NUM_QUEUES = 1;

  // Constructor
  virtio_block_t(simif_t* sim,
                 abstract_interrupt_controller_t* intctrl,
                 uint32_t interrupt_id,
                 const std::string& image_path,
                 bool readonly = false);

  virtual ~virtio_block_t();

  void tick(reg_t rtc_ticks) override;

protected:
  // virtio_base_t overrides
  uint32_t get_device_id() const override { return VIRTIO_ID_BLOCK; }
  uint64_t get_device_features() const override;
  void handle_driver_features(uint64_t features) override;
  void handle_queue_notify(uint32_t queue_idx) override;
  bool read_config(reg_t offset, size_t len, uint8_t* bytes) override;
  bool write_config(reg_t offset, size_t len, const uint8_t* bytes) override;
  uint32_t get_config_size() const override { return sizeof(virtio_blk_config); }
  void device_reset() override;

private:
  // Request processing
  void process_request_queue();

  // Block I/O operations
  bool do_read(uint64_t sector, uint8_t* buf, uint32_t num_sectors);
  bool do_write(uint64_t sector, const uint8_t* buf, uint32_t num_sectors);
  bool do_flush();
  bool do_get_id(char* buf, size_t len);

  // Configuration
  virtio_blk_config config;
  std::string image_path;
  std::string device_id;

  // File state
  int fd;
  bool readonly;
  uint64_t file_size;

  // Tick counter for polling
  uint64_t tick_count;
  static const uint64_t POLL_INTERVAL = 100;
};

// Factory functions for device registration
virtio_block_t* virtio_block_parse_from_fdt(const void* fdt, const sim_t* sim,
                                             reg_t* base, const std::vector<std::string>& sargs);
std::string virtio_block_generate_dts(const sim_t* sim, const std::vector<std::string>& sargs);

// Global configuration
void set_virtio_block_image_path(const std::string& path);
const std::string& get_virtio_block_image_path();
void set_virtio_block_readonly(bool readonly);
bool get_virtio_block_readonly();
bool is_virtio_block_enabled();

#endif // _RISCV_VIRTIO_BLOCK_H
