// See LICENSE for license details.
#ifndef _RISCV_VIRTIO_RNG_H
#define _RISCV_VIRTIO_RNG_H

#include "virtio.h"

// VirtIO RNG device ID
#define VIRTIO_ID_RNG 4

class virtio_rng_t : public virtio_base_t {
public:
  static const uint32_t QUEUE_REQUEST = 0;
  static const uint32_t NUM_QUEUES = 1;

  virtio_rng_t(simif_t* sim,
               abstract_interrupt_controller_t* intctrl,
               uint32_t interrupt_id);
  virtual ~virtio_rng_t();

  void tick(reg_t rtc_ticks) override;

protected:
  uint32_t get_device_id() const override { return VIRTIO_ID_RNG; }
  uint64_t get_device_features() const override { return VIRTIO_F_VERSION_1; }
  void handle_driver_features(uint64_t features) override { (void)features; }
  void handle_queue_notify(uint32_t queue_idx) override;
  bool read_config(reg_t offset, size_t len, uint8_t* bytes) override;
  bool write_config(reg_t offset, size_t len, const uint8_t* bytes) override;
  uint32_t get_config_size() const override { return 0; }
  void device_reset() override;

private:
  void process_request_queue();
};

// Factory functions
virtio_rng_t* virtio_rng_parse_from_fdt(const void* fdt, const sim_t* sim,
                                         reg_t* base, const std::vector<std::string>& sargs);
std::string virtio_rng_generate_dts(const sim_t* sim, const std::vector<std::string>& sargs);

// Global configuration
void set_virtio_rng_enabled(bool enabled);
bool is_virtio_rng_enabled();

#endif // _RISCV_VIRTIO_RNG_H
