// See LICENSE for license details.
#include "slirp_net.h"

#ifdef HAVE_SLIRP

#include <arpa/inet.h>
#include <poll.h>
#include <time.h>
#include <cstring>
#include <cstdio>

#define MAX_POLL_FDS 64

slirp_net_t::slirp_net_t(const slirp_config_t& config)
  : config(config),
    slirp(nullptr),
    poll_fds(nullptr),
    poll_fd_count(0),
    poll_fd_capacity(MAX_POLL_FDS)
{
  poll_fds = new struct pollfd[poll_fd_capacity];
}

slirp_net_t::~slirp_net_t() {
  if (slirp) {
    slirp_cleanup(slirp);
    slirp = nullptr;
  }
  delete[] poll_fds;
}

// Static callbacks for SLIRP
ssize_t slirp_net_t::send_packet_cb(const void* buf, size_t len, void* opaque) {
  slirp_net_t* self = static_cast<slirp_net_t*>(opaque);
  if (self->rx_callback) {
    self->rx_callback(static_cast<const uint8_t*>(buf), len);
  }
  return len;
}

void slirp_net_t::guest_error_cb(const char* msg, void* opaque) {
  (void)opaque;
  fprintf(stderr, "SLIRP error: %s\n", msg);
}

int64_t slirp_net_t::clock_get_ns_cb(void* opaque) {
  (void)opaque;
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

void* slirp_net_t::timer_new_cb(SlirpTimerCb cb_func, void* cb_opaque, void* opaque) {
  slirp_net_t* self = static_cast<slirp_net_t*>(opaque);

  // Find a free timer slot or create new one
  for (size_t i = 0; i < self->timers.size(); i++) {
    if (!self->timers[i].active) {
      self->timers[i].cb = cb_func;
      self->timers[i].cb_opaque = cb_opaque;
      self->timers[i].expire_time = -1;
      self->timers[i].active = true;
      return reinterpret_cast<void*>(i + 1); // 1-based index
    }
  }

  // Add new timer
  timer_entry entry;
  entry.cb = cb_func;
  entry.cb_opaque = cb_opaque;
  entry.expire_time = -1;
  entry.active = true;
  self->timers.push_back(entry);
  return reinterpret_cast<void*>(self->timers.size()); // 1-based index
}

void slirp_net_t::timer_free_cb(void* timer, void* opaque) {
  slirp_net_t* self = static_cast<slirp_net_t*>(opaque);
  size_t idx = reinterpret_cast<size_t>(timer) - 1;
  if (idx < self->timers.size()) {
    self->timers[idx].active = false;
  }
}

void slirp_net_t::timer_mod_cb(void* timer, int64_t expire_time, void* opaque) {
  slirp_net_t* self = static_cast<slirp_net_t*>(opaque);
  size_t idx = reinterpret_cast<size_t>(timer) - 1;
  if (idx < self->timers.size()) {
    self->timers[idx].expire_time = expire_time;
  }
}

void slirp_net_t::register_poll_fd_cb(int fd, void* opaque) {
  (void)fd;
  (void)opaque;
  // We manage poll fds ourselves in poll()
}

void slirp_net_t::unregister_poll_fd_cb(int fd, void* opaque) {
  (void)fd;
  (void)opaque;
}

void slirp_net_t::notify_cb(void* opaque) {
  (void)opaque;
}

int slirp_net_t::add_poll_cb(int fd, int events, void* opaque) {
  slirp_net_t* self = static_cast<slirp_net_t*>(opaque);
  if (self->poll_fd_count < self->poll_fd_capacity) {
    self->poll_fds[self->poll_fd_count].fd = fd;
    self->poll_fds[self->poll_fd_count].events = 0;
    if (events & SLIRP_POLL_IN) self->poll_fds[self->poll_fd_count].events |= POLLIN;
    if (events & SLIRP_POLL_OUT) self->poll_fds[self->poll_fd_count].events |= POLLOUT;
    self->poll_fds[self->poll_fd_count].revents = 0;
    return self->poll_fd_count++;
  }
  return -1;
}

int slirp_net_t::get_revents_cb(int idx, void* opaque) {
  slirp_net_t* self = static_cast<slirp_net_t*>(opaque);
  if (idx < 0 || idx >= self->poll_fd_count) return 0;
  int revents = 0;
  if (self->poll_fds[idx].revents & POLLIN) revents |= SLIRP_POLL_IN;
  if (self->poll_fds[idx].revents & POLLOUT) revents |= SLIRP_POLL_OUT;
  if (self->poll_fds[idx].revents & POLLERR) revents |= SLIRP_POLL_ERR;
  if (self->poll_fds[idx].revents & POLLHUP) revents |= SLIRP_POLL_HUP;
  return revents;
}

bool slirp_net_t::init() {
  SlirpConfig cfg;
  memset(&cfg, 0, sizeof(cfg));

  cfg.version = 1;
  cfg.restricted = false;
  cfg.in_enabled = true;
  cfg.vnetwork.s_addr = inet_addr(config.net_addr.c_str());
  cfg.vnetmask.s_addr = inet_addr(config.net_mask.c_str());
  cfg.vhost.s_addr = inet_addr(config.host_addr.c_str());
  cfg.vdhcp_start.s_addr = inet_addr(config.guest_addr.c_str());
  cfg.vnameserver.s_addr = inet_addr(config.dns_addr.c_str());

  static const SlirpCb callbacks = {
    .send_packet = send_packet_cb,
    .guest_error = guest_error_cb,
    .clock_get_ns = clock_get_ns_cb,
    .timer_new = timer_new_cb,
    .timer_free = timer_free_cb,
    .timer_mod = timer_mod_cb,
    .register_poll_fd = register_poll_fd_cb,
    .unregister_poll_fd = unregister_poll_fd_cb,
    .notify = notify_cb,
  };

  slirp = slirp_new(&cfg, &callbacks, this);
  if (!slirp) {
    fprintf(stderr, "slirp_net: failed to initialize SLIRP\n");
    return false;
  }

  // Add port forwarding
  struct in_addr host_addr_s = {.s_addr = INADDR_ANY};
  struct in_addr guest_addr_s = {.s_addr = inet_addr(config.guest_addr.c_str())};

  if (slirp_add_hostfwd(slirp, 0, host_addr_s, config.host_port,
                        guest_addr_s, config.guest_port) < 0) {
    fprintf(stderr, "slirp_net: warning - failed to add port forwarding\n");
  } else if (config.debug) {
    fprintf(stderr, "slirp_net: port forwarding localhost:%d -> %s:%d\n",
            config.host_port, config.guest_addr.c_str(), config.guest_port);
  }

  if (config.debug) {
    fprintf(stderr, "slirp_net: initialized (network: %s, gateway: %s)\n",
            config.net_addr.c_str(), config.host_addr.c_str());
  }

  return true;
}

void slirp_net_t::send_packet(const uint8_t* data, size_t len) {
  if (slirp) {
    slirp_input(slirp, data, len);
  }
}

void slirp_net_t::poll(int timeout_ms) {
  if (!slirp) return;

  // Reset poll fds
  poll_fd_count = 0;

  // Let SLIRP add its FDs
  uint32_t timeout = timeout_ms;
  slirp_pollfds_fill(slirp, &timeout, add_poll_cb, this);

  // Poll
  int ret = ::poll(poll_fds, poll_fd_count, timeout);

  // Let SLIRP process events
  slirp_pollfds_poll(slirp, ret <= 0, get_revents_cb, this);

  // Process expired timers
  int64_t now = clock_get_ns_cb(nullptr);
  for (auto& timer : timers) {
    if (timer.active && timer.expire_time >= 0 && timer.expire_time <= now) {
      timer.expire_time = -1;
      if (timer.cb) {
        timer.cb(timer.cb_opaque);
      }
    }
  }
}

#endif // HAVE_SLIRP
