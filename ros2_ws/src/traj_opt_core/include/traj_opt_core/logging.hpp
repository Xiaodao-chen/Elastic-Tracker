#pragma once

#include <iostream>
#include <sstream>

namespace traj_opt_core {

inline void log_error(const std::string& msg) {
  std::cerr << "[traj_opt_core][ERROR] " << msg << std::endl;
}

inline void log_warn(const std::string& msg) {
  std::cerr << "[traj_opt_core][WARN] " << msg << std::endl;
}

template <typename Fn>
inline void log_warn_stream(Fn&& fn) {
  std::ostringstream oss;
  fn(oss);
  log_warn(oss.str());
}

}  // namespace traj_opt_core

#define TRAJ_OPT_CORE_ERROR(MSG) ::traj_opt_core::log_error((MSG))
#define TRAJ_OPT_CORE_WARN(MSG) ::traj_opt_core::log_warn((MSG))

// Usage:
// TRAJ_OPT_CORE_WARN_STREAM([&](auto& os){ os << \"x=\" << x; });
#define TRAJ_OPT_CORE_WARN_STREAM(STREAM_FN) ::traj_opt_core::log_warn_stream((STREAM_FN))

