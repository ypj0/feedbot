// MotionAuthority 核心：/wheel/cmd_vel 的唯一所有权与租约状态机。
//
// 约束（第一阶段契约）：
//  * 纯 C++17，不依赖 ROS、不分配堆内存、不做 I/O、不写日志，可在 50 Hz 控制周期内调用。
//  * 时间由调用方以 now_ns 注入（单调、非负、纳秒），本库不读时钟。
//  * 同一时刻只允许一个有效租约；第二个来源的申请直接拒绝，不抢占、不排队、不自动切换。
//  * 未持有租约、租约过期、显式释放、故障时输出零速度。
//  * 所有时间窗和限幅由调用方通过 Config 传入（来自契约配置，只定义一次）。
//    Config 故意没有默认值：全零配置无效，会使状态机直接进入 FAULT。
//
// 状态：IDLE / ACTIVE / REVOKED / FAULT。
// “被拒绝”是事件而非状态：第二个来源被拒绝时，原所有者的租约不受影响，状态仍为 ACTIVE；
// 拒绝原因与计数通过 Status 上报。
#pragma once

#include <cmath>
#include <cstdint>

namespace feedbot::motion_authority {

enum class Source : std::uint8_t { None = 0, Manual = 1, Nav2 = 2, Dock = 3 };

enum class State : std::uint8_t { Idle = 0, Active = 1, Revoked = 2, Fault = 3 };

enum class Reason : std::uint8_t {
  None = 0,
  Busy,             // 已有其他来源持有租约
  AlreadyHeld,      // 同一来源重复申请
  InvalidSource,
  NoLease,          // 当前没有有效租约
  WrongLease,
  WrongSource,
  BadSequence,      // 序号未严格递增
  BadValue,         // 非有限值或超限
  StaleStamp,
  FutureStamp,
  Expired,          // 租约超时被撤销
  Released,         // 所有者显式释放
  FaultActive,      // 处于 FAULT，拒绝一切申请
  ClockRegression,  // 时间倒退或为负
  BadConfig,
  DownstreamFault   // 下游控制器异常（由外部上报）
};

struct Twist {
  double linear_x = 0.0;
  double angular_z = 0.0;
};

struct Config {
  std::int64_t lease_timeout_ns;    // 有效命令的续约窗口；超过即撤销租约
  std::int64_t max_stamp_age_ns;    // 命令时间戳允许落后 now 的最大值
  std::int64_t max_stamp_lead_ns;   // 命令时间戳允许领先 now 的最大值
  double max_linear;                // |linear_x| 上限，超限拒绝而不是裁剪
  double max_angular;               // |angular_z| 上限
};

struct Request {
  Source source;
  std::uint64_t lease_id;
  std::uint64_t seq;
  std::int64_t stamp_ns;
  Twist twist;
};

struct Grant {
  bool granted;
  std::uint64_t lease_id;
  Reason reason;
};

struct Output {
  Twist cmd;
  State state;
  Source owner;
};

struct Status {
  State state;
  Source owner;
  std::uint64_t lease_id;
  Reason last_reject;
  Reason last_revoke;
  Reason fault;
  std::int64_t input_age_ns;       // 距最近一次被接受命令的时间；本租约尚无命令为 -1
  std::int64_t lease_remaining_ns; // 非 ACTIVE 为 0；状态反映最近一次调用时刻，不会自行过期
  bool output_zero;
  std::uint64_t reject_count;
  std::uint64_t revoke_count;
  std::uint64_t fault_count;
};

inline bool config_valid(const Config& c) noexcept {
  return c.lease_timeout_ns > 0 && c.max_stamp_age_ns > 0 && c.max_stamp_lead_ns > 0 &&
         std::isfinite(c.max_linear) && c.max_linear > 0.0 &&
         std::isfinite(c.max_angular) && c.max_angular > 0.0;
}

inline bool source_valid(Source s) noexcept {
  return s == Source::Manual || s == Source::Nav2 || s == Source::Dock;
}

class Authority {
 public:
  explicit Authority(const Config& config) noexcept : cfg_(config) {
    if (!config_valid(config)) enter_fault(Reason::BadConfig);
  }

  // 申请租约。成功后调用方必须在 lease_timeout_ns 内提交有效命令，否则租约被撤销。
  Grant acquire(Source source, std::int64_t now_ns) noexcept {
    if (!check_clock(now_ns)) return {false, 0, reject(Reason::ClockRegression)};
    if (state_ == State::Fault) return {false, 0, reject(Reason::FaultActive)};
    if (!source_valid(source)) return {false, 0, reject(Reason::InvalidSource)};
    expire_if_needed(now_ns);
    if (state_ == State::Active) {
      return {false, 0, reject(owner_ == source ? Reason::AlreadyHeld : Reason::Busy)};
    }
    state_ = State::Active;
    owner_ = source;
    lease_id_ = ++next_lease_id_;
    last_seq_ = 0;
    cmd_ = Twist{};
    last_rx_ns_ = -1;
    deadline_ns_ = now_ns + cfg_.lease_timeout_ns;
    return {true, lease_id_, Reason::None};
  }

  // 提交运动命令。只有被接受的命令才会续约；被拒绝的命令不改变已保持的命令，也不续约。
  Reason submit(const Request& r, std::int64_t now_ns) noexcept {
    if (!check_clock(now_ns)) return reject(Reason::ClockRegression);
    if (state_ == State::Fault) return reject(Reason::FaultActive);
    expire_if_needed(now_ns);
    if (state_ != State::Active) return reject(Reason::NoLease);
    if (r.lease_id != lease_id_) return reject(Reason::WrongLease);
    if (r.source != owner_) return reject(Reason::WrongSource);
    if (r.seq <= last_seq_) return reject(Reason::BadSequence);
    if (!std::isfinite(r.twist.linear_x) || !std::isfinite(r.twist.angular_z) ||
        std::fabs(r.twist.linear_x) > cfg_.max_linear ||
        std::fabs(r.twist.angular_z) > cfg_.max_angular) {
      return reject(Reason::BadValue);
    }
    if (r.stamp_ns <= 0) return reject(Reason::StaleStamp);
    if (r.stamp_ns > now_ns) {
      if (r.stamp_ns - now_ns > cfg_.max_stamp_lead_ns) return reject(Reason::FutureStamp);
    } else if (now_ns - r.stamp_ns > cfg_.max_stamp_age_ns) {
      return reject(Reason::StaleStamp);
    }
    cmd_ = r.twist;
    last_seq_ = r.seq;
    last_rx_ns_ = now_ns;
    deadline_ns_ = now_ns + cfg_.lease_timeout_ns;
    return Reason::None;
  }

  // 所有者显式释放租约；释放后进入 REVOKED 并输出零速度，不会自动交给其他来源。
  Reason release(std::uint64_t lease_id, std::int64_t now_ns) noexcept {
    if (!check_clock(now_ns)) return reject(Reason::ClockRegression);
    if (state_ == State::Fault) return reject(Reason::FaultActive);
    expire_if_needed(now_ns);
    if (state_ != State::Active) return reject(Reason::NoLease);
    if (lease_id != lease_id_) return reject(Reason::WrongLease);
    revoke(Reason::Released);
    return Reason::None;
  }

  // 控制周期调用：处理过期并给出本周期应发布的速度。
  Output step(std::int64_t now_ns) noexcept {
    if (check_clock(now_ns)) expire_if_needed(now_ns);
    Output out{};
    out.state = state_;
    out.owner = owner_;
    if (state_ == State::Active) out.cmd = cmd_;
    return out;
  }

  // 外部上报下游异常（控制器离线、输出被阻塞等）。进入 FAULT 后只有 clear_fault 能退出。
  void report_fault(Reason reason, std::int64_t now_ns) noexcept {
    check_clock(now_ns);
    if (state_ != State::Fault) enter_fault(reason == Reason::None ? Reason::DownstreamFault : reason);
  }

  // 显式清除故障，回到 IDLE，并以 now_ns 重置时间基准。配置无效时不可清除。
  bool clear_fault(std::int64_t now_ns) noexcept {
    if (state_ != State::Fault || !config_valid(cfg_) || now_ns < 0) return false;
    state_ = State::Idle;
    fault_ = Reason::None;
    last_now_ns_ = now_ns;
    have_time_ = true;
    return true;
  }

  // 供上层记录“到达核心之前就能判定无效”的请求（例如 Twist 中不允许的分量）。
  // 只增加拒绝计数与原因，不改变状态、所有者或输出。
  Reason note_reject(Reason reason) noexcept { return reject(reason); }

  Status status(std::int64_t now_ns) const noexcept {
    Status s{};
    s.state = state_;
    s.owner = owner_;
    s.lease_id = state_ == State::Active ? lease_id_ : 0;
    s.last_reject = last_reject_;
    s.last_revoke = last_revoke_;
    s.fault = fault_;
    if (state_ == State::Active) {
      s.lease_remaining_ns = deadline_ns_ > now_ns ? deadline_ns_ - now_ns : 0;
      s.output_zero = cmd_.linear_x == 0.0 && cmd_.angular_z == 0.0;
    } else {
      s.lease_remaining_ns = 0;
      s.output_zero = true;
    }
    s.input_age_ns = (state_ == State::Active && last_rx_ns_ >= 0)
                         ? (now_ns > last_rx_ns_ ? now_ns - last_rx_ns_ : 0)
                         : -1;
    s.reject_count = reject_count_;
    s.revoke_count = revoke_count_;
    s.fault_count = fault_count_;
    return s;
  }

 private:
  bool check_clock(std::int64_t now_ns) noexcept {
    if (now_ns < 0 || (have_time_ && now_ns < last_now_ns_)) {
      if (state_ != State::Fault) enter_fault(Reason::ClockRegression);
      return false;
    }
    last_now_ns_ = now_ns;
    have_time_ = true;
    return true;
  }

  Reason reject(Reason r) noexcept {
    last_reject_ = r;
    ++reject_count_;
    return r;
  }

  void expire_if_needed(std::int64_t now_ns) noexcept {
    if (state_ == State::Active && now_ns >= deadline_ns_) revoke(Reason::Expired);
  }

  void revoke(Reason r) noexcept {
    state_ = State::Revoked;
    owner_ = Source::None;
    cmd_ = Twist{};
    last_revoke_ = r;
    ++revoke_count_;
  }

  void enter_fault(Reason r) noexcept {
    state_ = State::Fault;
    owner_ = Source::None;
    cmd_ = Twist{};
    fault_ = r;
    ++fault_count_;
  }

  Config cfg_;
  State state_ = State::Idle;
  Source owner_ = Source::None;
  std::uint64_t lease_id_ = 0;
  std::uint64_t next_lease_id_ = 0;
  std::uint64_t last_seq_ = 0;
  Twist cmd_{};
  std::int64_t deadline_ns_ = 0;
  std::int64_t last_rx_ns_ = -1;
  std::int64_t last_now_ns_ = 0;
  bool have_time_ = false;
  Reason last_reject_ = Reason::None;
  Reason last_revoke_ = Reason::None;
  Reason fault_ = Reason::None;
  std::uint64_t reject_count_ = 0;
  std::uint64_t revoke_count_ = 0;
  std::uint64_t fault_count_ = 0;
};

}  // namespace feedbot::motion_authority
