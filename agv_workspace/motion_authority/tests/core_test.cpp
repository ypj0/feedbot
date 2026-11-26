// MotionAuthority 核心库单测：独立可执行文件，沿用 chassis_ros2/tests 的风格。
// 失败时打印文件行号并返回非零；全部通过时打印 PASS 行。
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <type_traits>
#include <utility>

#include "feedbot_motion_authority/authority.hpp"

using namespace feedbot::motion_authority;

namespace {

int g_failures = 0;

#define CHECK(cond)                                                          \
  do {                                                                       \
    if (!(cond)) {                                                           \
      std::fprintf(stderr, "FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);   \
      ++g_failures;                                                          \
    }                                                                        \
  } while (0)

constexpr std::int64_t MS = 1'000'000;

Config cfg() {
  Config c{};
  c.lease_timeout_ns = 200 * MS;
  c.max_stamp_age_ns = 100 * MS;
  c.max_stamp_lead_ns = 50 * MS;
  c.max_linear = 0.5;
  c.max_angular = 1.0;
  return c;
}

Request req(Source s, std::uint64_t lease, std::uint64_t seq, std::int64_t stamp, double vx,
            double wz) {
  Request r{};
  r.source = s;
  r.lease_id = lease;
  r.seq = seq;
  r.stamp_ns = stamp;
  r.twist.linear_x = vx;
  r.twist.angular_z = wz;
  return r;
}

bool is_zero(const Twist& t) {
  return t.linear_x == 0.0 && t.angular_z == 0.0 && !std::signbit(t.linear_x) &&
         !std::signbit(t.angular_z);
}

void test_idle_outputs_zero() {
  Authority a(cfg());
  for (int i = 1; i <= 5; ++i) {
    Output o = a.step(i * 20 * MS);
    CHECK(o.state == State::Idle);
    CHECK(o.owner == Source::None);
    CHECK(is_zero(o.cmd));
  }
  Status s = a.status(100 * MS);
  CHECK(s.output_zero);
  CHECK(s.input_age_ns == -1);
  CHECK(s.lease_remaining_ns == 0);
  CHECK(s.reject_count == 0 && s.revoke_count == 0 && s.fault_count == 0);
}

void test_grant_and_command_hold() {
  Authority a(cfg());
  const std::int64_t t0 = 1000 * MS;
  Grant g = a.acquire(Source::Manual, t0);
  CHECK(g.granted && g.lease_id == 1 && g.reason == Reason::None);
  Output o = a.step(t0 + 20 * MS);
  CHECK(o.state == State::Active && o.owner == Source::Manual);
  CHECK(is_zero(o.cmd));  // 尚无命令时仍输出零
  CHECK(a.submit(req(Source::Manual, g.lease_id, 1, t0 + 20 * MS, 0.2, 0.1), t0 + 20 * MS) ==
        Reason::None);
  // 命令保持：无新命令时按固定周期持续输出最近一次命令，直到租约过期。
  for (std::int64_t dt = 40; dt <= 200; dt += 20) {
    o = a.step(t0 + dt * MS);
    CHECK(o.state == State::Active);
    CHECK(o.cmd.linear_x == 0.2 && o.cmd.angular_z == 0.1);
  }
  Status s = a.status(t0 + 100 * MS);
  CHECK(s.input_age_ns == 80 * MS);
  CHECK(!s.output_zero);
}

void test_concurrent_rejected_no_preempt_no_queue() {
  Authority a(cfg());
  const std::int64_t t0 = 1000 * MS;
  Grant m = a.acquire(Source::Manual, t0);
  CHECK(m.granted);
  CHECK(a.submit(req(Source::Manual, m.lease_id, 1, t0, 0.3, 0.0), t0) == Reason::None);

  Grant n = a.acquire(Source::Nav2, t0 + 10 * MS);
  CHECK(!n.granted && n.lease_id == 0 && n.reason == Reason::Busy);
  Grant again = a.acquire(Source::Manual, t0 + 10 * MS);
  CHECK(!again.granted && again.reason == Reason::AlreadyHeld);
  Grant d = a.acquire(Source::Dock, t0 + 10 * MS);
  CHECK(!d.granted && d.reason == Reason::Busy);

  // 被拒绝不影响原所有者。
  Output o = a.step(t0 + 20 * MS);
  CHECK(o.state == State::Active && o.owner == Source::Manual && o.cmd.linear_x == 0.3);
  Status s = a.status(t0 + 20 * MS);
  CHECK(s.reject_count == 3 && s.last_reject == Reason::Busy);

  // 其他来源不能借用别人的租约，也不能猜租约号。
  CHECK(a.submit(req(Source::Nav2, m.lease_id, 2, t0 + 20 * MS, 0.1, 0.0), t0 + 20 * MS) ==
        Reason::WrongSource);
  CHECK(a.submit(req(Source::Nav2, 0, 2, t0 + 20 * MS, 0.1, 0.0), t0 + 20 * MS) ==
        Reason::WrongLease);

  // 无排队、无自动切换：Manual 释放后进入 REVOKED 并输出零，Nav2 必须重新申请。
  CHECK(a.release(m.lease_id, t0 + 30 * MS) == Reason::None);
  o = a.step(t0 + 40 * MS);
  CHECK(o.state == State::Revoked && o.owner == Source::None && is_zero(o.cmd));
  Grant n2 = a.acquire(Source::Nav2, t0 + 50 * MS);
  CHECK(n2.granted && n2.lease_id == 2);
}

void test_invalid_source_rejected() {
  Authority a(cfg());
  CHECK(a.acquire(Source::None, 10 * MS).reason == Reason::InvalidSource);
  CHECK(a.acquire(static_cast<Source>(9), 10 * MS).reason == Reason::InvalidSource);
  CHECK(a.step(20 * MS).state == State::Idle);
}

void test_submit_validation() {
  Authority a(cfg());
  const std::int64_t t0 = 1000 * MS;
  CHECK(a.submit(req(Source::Manual, 1, 1, t0, 0.1, 0.0), t0) == Reason::NoLease);  // 未申请

  Grant g = a.acquire(Source::Manual, t0);
  const std::uint64_t id = g.lease_id;
  CHECK(a.submit(req(Source::Manual, id + 1, 1, t0, 0.1, 0.0), t0) == Reason::WrongLease);
  CHECK(a.submit(req(Source::Manual, id, 0, t0, 0.1, 0.0), t0) == Reason::BadSequence);
  CHECK(a.submit(req(Source::Manual, id, 1, t0, 0.1, 0.0), t0) == Reason::None);
  CHECK(a.submit(req(Source::Manual, id, 1, t0, 0.1, 0.0), t0) == Reason::BadSequence);  // 重复
  CHECK(a.submit(req(Source::Manual, id, 5, t0, 0.1, 0.0), t0) == Reason::None);          // 允许跳号
  CHECK(a.submit(req(Source::Manual, id, 4, t0, 0.1, 0.0), t0) == Reason::BadSequence);  // 倒退

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  CHECK(a.submit(req(Source::Manual, id, 6, t0, nan, 0.0), t0) == Reason::BadValue);
  CHECK(a.submit(req(Source::Manual, id, 6, t0, 0.0, nan), t0) == Reason::BadValue);
  CHECK(a.submit(req(Source::Manual, id, 6, t0, inf, 0.0), t0) == Reason::BadValue);
  CHECK(a.submit(req(Source::Manual, id, 6, t0, -inf, 0.0), t0) == Reason::BadValue);
  CHECK(a.submit(req(Source::Manual, id, 6, t0, 0.5000001, 0.0), t0) == Reason::BadValue);
  CHECK(a.submit(req(Source::Manual, id, 6, t0, 0.0, -1.0000001), t0) == Reason::BadValue);
  // note_reject 只记账，不影响租约与输出。
  {
    const std::uint64_t before = a.status(t0).reject_count;
    CHECK(a.note_reject(Reason::BadValue) == Reason::BadValue);
    Status ns = a.status(t0);
    CHECK(ns.reject_count == before + 1 && ns.last_reject == Reason::BadValue);
    CHECK(ns.state == State::Active && ns.owner == Source::Manual && ns.lease_id == id);
  }
  // 被拒绝的命令不推进序号：同一个序号随后可被接受；恰在限幅上允许。
  CHECK(a.submit(req(Source::Manual, id, 6, t0, 0.5, -1.0), t0) == Reason::None);

  // 时间戳边界：落后恰为 max_age 允许，多 1 ns 拒绝；领先同理。
  CHECK(a.submit(req(Source::Manual, id, 7, t0 - 100 * MS, 0.1, 0.0), t0) == Reason::None);
  CHECK(a.submit(req(Source::Manual, id, 8, t0 - 100 * MS - 1, 0.1, 0.0), t0) ==
        Reason::StaleStamp);
  CHECK(a.submit(req(Source::Manual, id, 8, t0 + 50 * MS, 0.1, 0.0), t0) == Reason::None);
  CHECK(a.submit(req(Source::Manual, id, 9, t0 + 50 * MS + 1, 0.1, 0.0), t0) ==
        Reason::FutureStamp);
  CHECK(a.submit(req(Source::Manual, id, 9, 0, 0.1, 0.0), t0) == Reason::StaleStamp);
  CHECK(a.submit(req(Source::Manual, id, 9, -5, 0.1, 0.0), t0) == Reason::StaleStamp);
  CHECK(a.submit(req(Source::Manual, id, 9, std::numeric_limits<std::int64_t>::min(), 0.1, 0.0),
                 t0) == Reason::StaleStamp);
}

void test_rejected_submit_keeps_command_and_does_not_renew() {
  Authority a(cfg());
  const std::int64_t t0 = 1000 * MS;
  Grant g = a.acquire(Source::Manual, t0);
  CHECK(a.submit(req(Source::Manual, g.lease_id, 1, t0, 0.2, 0.0), t0) == Reason::None);
  const std::int64_t remaining_before = a.status(t0 + 100 * MS).lease_remaining_ns;
  CHECK(a.submit(req(Source::Manual, g.lease_id, 2, t0 + 100 * MS, 9.0, 0.0), t0 + 100 * MS) ==
        Reason::BadValue);
  Status s = a.status(t0 + 100 * MS);
  CHECK(s.lease_remaining_ns == remaining_before);  // 未续约
  CHECK(s.input_age_ns == 100 * MS);                // 输入年龄未被重置
  CHECK(a.step(t0 + 100 * MS).cmd.linear_x == 0.2);  // 保持原命令
}

void test_lease_timeout_and_renewal() {
  Authority a(cfg());
  const std::int64_t t0 = 1000 * MS;
  Grant g = a.acquire(Source::Nav2, t0);
  // 申请后没有任何命令：窗口内保持 ACTIVE，恰到期时撤销。
  CHECK(a.step(t0 + 199 * MS).state == State::Active);
  Output o = a.step(t0 + 200 * MS);
  CHECK(o.state == State::Revoked && is_zero(o.cmd));
  CHECK(a.status(t0 + 200 * MS).last_revoke == Reason::Expired);
  CHECK(a.status(t0 + 200 * MS).revoke_count == 1);

  // 续约：窗口内被接受的命令把截止时间推到 now + timeout。
  Grant g2 = a.acquire(Source::Nav2, t0 + 300 * MS);
  CHECK(g2.granted && g2.lease_id == g.lease_id + 1);
  CHECK(a.submit(req(Source::Nav2, g2.lease_id, 1, t0 + 490 * MS, 0.2, 0.0), t0 + 490 * MS) ==
        Reason::None);
  CHECK(a.step(t0 + 689 * MS).state == State::Active);
  o = a.step(t0 + 690 * MS);
  CHECK(o.state == State::Revoked && is_zero(o.cmd));
  // 撤销后零输出持续，且不会自行恢复。
  for (int i = 1; i <= 5; ++i) {
    o = a.step(t0 + 690 * MS + i * 20 * MS);
    CHECK(o.state == State::Revoked && is_zero(o.cmd));
  }
}

void test_submit_at_deadline_does_not_resurrect_lease() {
  Authority a(cfg());
  const std::int64_t t0 = 1000 * MS;
  Grant g = a.acquire(Source::Manual, t0);
  CHECK(a.submit(req(Source::Manual, g.lease_id, 1, t0 + 200 * MS, 0.2, 0.0), t0 + 200 * MS) ==
        Reason::NoLease);
  CHECK(a.step(t0 + 200 * MS).state == State::Revoked);
}

void test_release() {
  Authority a(cfg());
  const std::int64_t t0 = 1000 * MS;
  Grant g = a.acquire(Source::Dock, t0);
  CHECK(a.submit(req(Source::Dock, g.lease_id, 1, t0, 0.2, 0.0), t0) == Reason::None);
  CHECK(a.release(g.lease_id + 7, t0 + 10 * MS) == Reason::WrongLease);
  CHECK(a.step(t0 + 10 * MS).state == State::Active);  // 错误释放不影响租约
  CHECK(a.release(g.lease_id, t0 + 20 * MS) == Reason::None);
  Output o = a.step(t0 + 20 * MS);
  CHECK(o.state == State::Revoked && o.owner == Source::None && is_zero(o.cmd));
  CHECK(a.status(t0 + 20 * MS).last_revoke == Reason::Released);
  CHECK(a.submit(req(Source::Dock, g.lease_id, 2, t0 + 30 * MS, 0.2, 0.0), t0 + 30 * MS) ==
        Reason::NoLease);
  CHECK(a.release(g.lease_id, t0 + 30 * MS) == Reason::NoLease);
}

void test_stale_lease_after_reacquire() {
  Authority a(cfg());
  const std::int64_t t0 = 1000 * MS;
  Grant m = a.acquire(Source::Manual, t0);
  CHECK(a.release(m.lease_id, t0 + 10 * MS) == Reason::None);
  Grant n = a.acquire(Source::Nav2, t0 + 20 * MS);
  CHECK(n.granted && n.lease_id > m.lease_id);
  // 旧所有者拿旧租约号不能发命令，也不能释放新租约。
  CHECK(a.submit(req(Source::Manual, m.lease_id, 1, t0 + 30 * MS, 0.1, 0.0), t0 + 30 * MS) ==
        Reason::WrongLease);
  CHECK(a.release(m.lease_id, t0 + 30 * MS) == Reason::WrongLease);
  CHECK(a.step(t0 + 30 * MS).owner == Source::Nav2);
  // 新租约序号从头计：新所有者的 seq=1 可接受。
  CHECK(a.submit(req(Source::Nav2, n.lease_id, 1, t0 + 30 * MS, 0.1, 0.0), t0 + 30 * MS) ==
        Reason::None);
}

void test_fault_blocks_everything_until_cleared() {
  Authority a(cfg());
  const std::int64_t t0 = 1000 * MS;
  Grant g = a.acquire(Source::Manual, t0);
  CHECK(a.submit(req(Source::Manual, g.lease_id, 1, t0, 0.3, 0.2), t0) == Reason::None);
  a.report_fault(Reason::DownstreamFault, t0 + 10 * MS);
  Output o = a.step(t0 + 20 * MS);
  CHECK(o.state == State::Fault && o.owner == Source::None && is_zero(o.cmd));
  CHECK(a.status(t0 + 20 * MS).fault == Reason::DownstreamFault);
  CHECK(!a.acquire(Source::Manual, t0 + 30 * MS).granted);
  CHECK(a.acquire(Source::Manual, t0 + 30 * MS).reason == Reason::FaultActive);
  CHECK(a.submit(req(Source::Manual, g.lease_id, 2, t0 + 30 * MS, 0.1, 0.0), t0 + 30 * MS) ==
        Reason::FaultActive);
  CHECK(a.release(g.lease_id, t0 + 30 * MS) == Reason::FaultActive);
  CHECK(is_zero(a.step(t0 + 40 * MS).cmd));

  CHECK(a.clear_fault(t0 + 50 * MS));
  CHECK(a.step(t0 + 60 * MS).state == State::Idle);
  CHECK(!a.clear_fault(t0 + 70 * MS));  // 不处于 FAULT 时清除无效
  Grant g2 = a.acquire(Source::Manual, t0 + 80 * MS);
  CHECK(g2.granted && g2.lease_id > g.lease_id);
  // 不带原因的上报按下游故障处理；FAULT 中重复上报不改变首个原因。
  Authority b(cfg());
  b.report_fault(Reason::None, 10 * MS);
  CHECK(b.status(10 * MS).fault == Reason::DownstreamFault);
  b.report_fault(Reason::ClockRegression, 20 * MS);
  CHECK(b.status(20 * MS).fault == Reason::DownstreamFault && b.status(20 * MS).fault_count == 1);
}

void test_clock_regression_and_negative_time() {
  Authority a(cfg());
  Grant g = a.acquire(Source::Manual, 1000 * MS);
  CHECK(g.granted);
  Output o = a.step(999 * MS);  // 时间倒退
  CHECK(o.state == State::Fault && is_zero(o.cmd));
  CHECK(a.status(999 * MS).fault == Reason::ClockRegression);
  CHECK(a.clear_fault(500 * MS));  // 清除时以新时间重置基准
  CHECK(a.step(501 * MS).state == State::Idle);

  Authority b(cfg());
  CHECK(b.step(-1).state == State::Fault);  // 负时间
  CHECK(b.status(0).fault == Reason::ClockRegression);
  Authority c(cfg());
  CHECK(c.acquire(Source::Manual, 100 * MS).granted);
  CHECK(c.acquire(Source::Nav2, 50 * MS).reason == Reason::ClockRegression);
  CHECK(c.step(120 * MS).state == State::Fault);  // 已进入 FAULT，且无自动恢复
}

void test_bad_config_starts_in_fault() {
  Authority a(Config{});
  CHECK(a.step(10 * MS).state == State::Fault);
  CHECK(a.status(10 * MS).fault == Reason::BadConfig);
  CHECK(a.acquire(Source::Manual, 20 * MS).reason == Reason::FaultActive);
  CHECK(!a.clear_fault(30 * MS));
  CHECK(a.step(40 * MS).state == State::Fault);

  // 逐项破坏每个配置字段，均应无效。
  const double nan = std::numeric_limits<double>::quiet_NaN();
  Config c = cfg();
  CHECK(config_valid(c));
  c = cfg(); c.lease_timeout_ns = 0;  CHECK(!config_valid(c));
  c = cfg(); c.lease_timeout_ns = -1; CHECK(!config_valid(c));
  c = cfg(); c.max_stamp_age_ns = 0;  CHECK(!config_valid(c));
  c = cfg(); c.max_stamp_lead_ns = 0; CHECK(!config_valid(c));
  c = cfg(); c.max_linear = 0.0;      CHECK(!config_valid(c));
  c = cfg(); c.max_linear = nan;      CHECK(!config_valid(c));
  c = cfg(); c.max_angular = -1.0;    CHECK(!config_valid(c));
  c = cfg(); c.max_angular = std::numeric_limits<double>::infinity(); CHECK(!config_valid(c));
}

void test_no_heap_and_trivial_types() {
  static_assert(std::is_trivially_copyable<Authority>::value, "Authority must be POD-like");
  static_assert(std::is_trivially_destructible<Authority>::value, "no destructor work");
  static_assert(std::is_trivially_copyable<Request>::value, "");
  static_assert(std::is_trivially_copyable<Output>::value, "");
  static_assert(std::is_trivially_copyable<Status>::value, "");
  static_assert(std::is_standard_layout<Request>::value, "");
  static_assert(noexcept(std::declval<Authority&>().step(0)), "step must not throw");
  static_assert(noexcept(std::declval<Authority&>().submit(Request{}, 0)), "submit must not throw");
  static_assert(sizeof(Authority) <= 256, "state must stay small and cache friendly");
  CHECK(true);
}

// ---- 确定性随机操作序列：每一步检查全局不变量 ------------------------------------------
struct Rng {
  std::uint64_t s;
  std::uint64_t next() {
    s = s * 6364136223846793005ULL + 1442695040888963407ULL;
    return s >> 33;
  }
  double unit() { return static_cast<double>(next() % 2001) / 1000.0 - 1.0; }  // [-1, 1]
};

void run_fuzz(std::uint64_t seed, int iterations) {
  const Config c = cfg();
  Authority a(c);
  Rng rng{seed};
  std::int64_t now = 1000 * MS;
  std::uint64_t known_lease = 0, max_granted = 0, seq = 0;
  const double nan = std::numeric_limits<double>::quiet_NaN();

  for (int i = 0; i < iterations; ++i) {
    now += static_cast<std::int64_t>(rng.next() % (30 * MS));
    if (rng.next() % 50 == 0) now += 250 * MS;  // 偶发长间隔，触发过期

    const State before = a.status(now).state;
    bool cleared = false;
    switch (rng.next() % 8) {
      case 0: {
        const Source s = static_cast<Source>(rng.next() % 5);
        Grant g = a.acquire(s, now);
        if (g.granted) {
          CHECK(g.lease_id > max_granted);  // 租约号严格递增、不复用
          max_granted = g.lease_id;
          known_lease = g.lease_id;
          seq = 0;
          CHECK(a.status(now).state == State::Active);
        } else {
          CHECK(g.lease_id == 0 && g.reason != Reason::None);
        }
        break;
      }
      case 1: case 2: case 3: {
        const std::uint64_t lease = (rng.next() % 4 == 0) ? rng.next() % 5 : known_lease;
        const Source s = (rng.next() % 5 == 0) ? static_cast<Source>(rng.next() % 4)
                                               : a.status(now).owner;
        const std::uint64_t q = (rng.next() % 6 == 0) ? rng.next() % 4 : ++seq;
        const std::int64_t stamp = now - static_cast<std::int64_t>(rng.next() % (130 * MS)) +
                                   static_cast<std::int64_t>(rng.next() % (60 * MS));
        double vx = rng.unit() * 0.6, wz = rng.unit() * 1.2;
        if (rng.next() % 17 == 0) vx = nan;
        a.submit(req(s, lease, q, stamp, vx, wz), now);
        break;
      }
      case 4:
        a.release((rng.next() % 3 == 0) ? rng.next() % 5 : known_lease, now);
        break;
      case 5:
        break;  // 仅 step
      case 6:
        if (rng.next() % 40 == 0) a.report_fault(Reason::DownstreamFault, now);
        break;
      default:
        if (rng.next() % 6 == 0) cleared = a.clear_fault(now);
        break;
    }

    const Output o = a.step(now);
    const Status s = a.status(now);

    // FAULT 只能经 clear_fault 退出。
    if (before == State::Fault && !cleared) CHECK(s.state == State::Fault);
    // 结构不变量：ACTIVE 恰有一个有效所有者；非 ACTIVE 无所有者且输出为零。
    if (s.state == State::Active) {
      CHECK(source_valid(s.owner) && s.lease_id != 0 && s.lease_remaining_ns > 0);
    } else {
      CHECK(s.owner == Source::None && s.lease_remaining_ns == 0 && is_zero(o.cmd));
    }
    // 输出不变量：非零输出必须来自 ACTIVE 且租约未过期、有限、未超限。
    if (!is_zero(o.cmd)) {
      CHECK(o.state == State::Active && source_valid(o.owner));
      CHECK(std::isfinite(o.cmd.linear_x) && std::fabs(o.cmd.linear_x) <= c.max_linear);
      CHECK(std::isfinite(o.cmd.angular_z) && std::fabs(o.cmd.angular_z) <= c.max_angular);
      CHECK(s.lease_remaining_ns > 0);
    }
    if (g_failures != 0) {
      std::fprintf(stderr, "fuzz seed=%llu iteration=%d now=%lld\n",
                   static_cast<unsigned long long>(seed), i, static_cast<long long>(now));
      return;
    }
  }
}

void test_fuzz_invariants() {
  for (std::uint64_t seed = 1; seed <= 6; ++seed) run_fuzz(seed, 100000);
}

}  // namespace

int main() {
  test_idle_outputs_zero();
  test_grant_and_command_hold();
  test_concurrent_rejected_no_preempt_no_queue();
  test_invalid_source_rejected();
  test_submit_validation();
  test_rejected_submit_keeps_command_and_does_not_renew();
  test_lease_timeout_and_renewal();
  test_submit_at_deadline_does_not_resurrect_lease();
  test_release();
  test_stale_lease_after_reacquire();
  test_fault_blocks_everything_until_cleared();
  test_clock_regression_and_negative_time();
  test_bad_config_starts_in_fault();
  test_no_heap_and_trivial_types();
  test_fuzz_invariants();
  if (g_failures != 0) {
    std::fprintf(stderr, "motion_authority_core_test: %d check(s) failed\n", g_failures);
    return 1;
  }
  std::puts("PASS idle zero output, lease grant and command hold");
  std::puts("PASS concurrent request rejected: no preempt, no queue, no auto switch");
  std::puts("PASS submit validation: lease/source/sequence/NaN/limits/stamp boundaries");
  std::puts("PASS timeout, renewal, release, stale lease after reacquire");
  std::puts("PASS fault blocks all until explicit clear; clock regression; bad config");
  std::puts("PASS trivially copyable, noexcept, small state");
  std::puts("PASS 6x100000 deterministic random operations with invariants");
  return 0;
}
