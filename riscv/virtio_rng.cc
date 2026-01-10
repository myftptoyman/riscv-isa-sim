// See LICENSE for license details.
#include "virtio_rng.h"
#include "sim.h"
#include "devices.h"
#include "abstract_device.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sstream>

// Base address for VirtIO RNG
#define VIRTIO_RNG_BASE 0x10004000

static bool debug_virtio_rng = false;

virtio_rng_t::virtio_rng_t(simif_t* sim,
                           abstract_interrupt_controller_t* intctrl,
                           uint32_t interrupt_id)
  : virtio_base_t(sim, intctrl, interrupt_id, NUM_QUEUES, 64)
{
  debug_virtio_rng = getenv("DEBUG_VIRTIO") != nullptr;
  if (debug_virtio_rng) {
    fprintf(stderr, "virtio-rng: initialized\n");
  }
}

virtio_rng_t::~virtio_rng_t() {
}

void virtio_rng_t::device_reset() {
  if (debug_virtio_rng) {
    fprintf(stderr, "virtio-rng: device reset\n");
  }
}

bool virtio_rng_t::read_config(reg_t offset, size_t len, uint8_t* bytes) {
  (void)offset;
  (void)len;
  (void)bytes;
  // No configuration space for RNG device
  return false;
}

bool virtio_rng_t::write_config(reg_t offset, size_t len, const uint8_t* bytes) {
  (void)offset;
  (void)len;
  (void)bytes;
  // No configuration space for RNG device
  return false;
}

void virtio_rng_t::handle_queue_notify(uint32_t queue_idx) {
  if (debug_virtio_rng) {
    fprintf(stderr, "virtio-rng: queue_notify queue=%u\n", queue_idx);
  }
  if (queue_idx == QUEUE_REQUEST) {
    process_request_queue();
  }
}

void virtio_rng_t::process_request_queue() {
  bool did_work = false;

  while (true) {
    std::vector<vring_desc> out_descs, in_descs;
    int head = queues[QUEUE_REQUEST].get_avail_buf(out_descs, in_descs);
    if (head < 0) {
      break;  // No more descriptors available
    }

    // VirtIO RNG: device writes random data to in_descs (device-writable buffers)
    size_t total_written = 0;
    for (const auto& desc : in_descs) {
      size_t len = desc.len;
      std::vector<uint8_t> random_data(len);

      // Read from /dev/urandom on host
      int fd = open("/dev/urandom", O_RDONLY);
      if (fd >= 0) {
        ssize_t n = read(fd, random_data.data(), len);
        close(fd);
        if (n > 0) {
          len = static_cast<size_t>(n);
        }
      } else {
        // Fallback: use rand()
        for (size_t i = 0; i < len; i++) {
          random_data[i] = rand() & 0xff;
        }
      }

      // Write to guest memory
      queues[QUEUE_REQUEST].write_guest_mem(desc.addr, random_data.data(), len);
      total_written += len;

      if (debug_virtio_rng) {
        fprintf(stderr, "virtio-rng: filled %zu bytes of random data\n", len);
      }
    }

    // Complete the request
    queues[QUEUE_REQUEST].put_used_buf(static_cast<uint16_t>(head), static_cast<uint32_t>(total_written));
    did_work = true;
  }

  // Raise interrupt to notify guest if we processed any requests
  if (did_work) {
    raise_interrupt(VIRTIO_INT_USED_RING);
  }
}

void virtio_rng_t::tick(reg_t rtc_ticks) {
  (void)rtc_ticks;
  // Nothing to do on tick for RNG device
}

// Global flag for enabling VirtIO RNG
static bool virtio_rng_enabled = false;

void set_virtio_rng_enabled(bool enabled) {
  virtio_rng_enabled = enabled;
  fprintf(stderr, "virtio-rng: %s\n", enabled ? "enabled" : "disabled");
}

bool is_virtio_rng_enabled() {
  return virtio_rng_enabled;
}

std::string virtio_rng_generate_dts(const sim_t* sim, const std::vector<std::string>& sargs) {
  (void)sargs;

  if (!virtio_rng_enabled) {
    return "";
  }

  std::ostringstream oss;
  reg_t base = VIRTIO_RNG_BASE;

  if (debug_virtio_rng) {
    fprintf(stderr, "virtio_rng_generate_dts: generating DTS node at 0x%lx\n", (unsigned long)base);
  }

  oss << "    virtio_rng@" << std::hex << base << " {\n"
      << "      compatible = \"virtio,mmio\";\n"
      << "      interrupt-parent = <&PLIC>;\n"
      << "      interrupts = <0x05>;\n"  // Interrupt ID 5
      << "      reg = <0x00 0x" << base << " 0x00 0x1000>;\n"
      << "    };\n";

  return oss.str();
}

virtio_rng_t* virtio_rng_parse_from_fdt(const void* fdt, const sim_t* sim,
                                         reg_t* base, const std::vector<std::string>& sargs) {
  (void)fdt;
  (void)sargs;

  if (!virtio_rng_enabled) {
    return nullptr;
  }

  *base = VIRTIO_RNG_BASE;

  // Get the interrupt controller (PLIC)
  abstract_interrupt_controller_t* intctrl = sim->get_intctrl();
  if (!intctrl) {
    fprintf(stderr, "virtio-rng: no interrupt controller found\n");
    return nullptr;
  }

  virtio_rng_t* dev = new virtio_rng_t(
    const_cast<sim_t*>(sim),
    intctrl,
    5  // Interrupt ID
  );

  fprintf(stderr, "virtio-rng: created device at 0x%lx\n", (unsigned long)*base);
  return dev;
}

// Device factory - uses REGISTER_BUILTIN_DEVICE macro from abstract_device.h
REGISTER_BUILTIN_DEVICE(virtio_rng, virtio_rng_parse_from_fdt, virtio_rng_generate_dts)
