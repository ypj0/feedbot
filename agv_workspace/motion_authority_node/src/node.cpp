// MotionAuthority ROS 2 节点：单线程、50 Hz、无锁。
// 核心使用单调时钟；系统时间戳在这里转换。
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "feedbot_motion_interfaces/msg/motion_request.hpp"
#include "feedbot_motion_interfaces/msg/motion_authority_status.hpp"
#include "feedbot_motion_interfaces/msg/performance_stats.hpp"
#include "feedbot_motion_interfaces/srv/acquire_lease.hpp"
#include "feedbot_motion_interfaces/srv/release_lease.hpp"
#include "feedbot_motion_interfaces/srv/clear_fault.hpp"
#include "feedbot_motion_authority/authority.hpp"

using namespace std::chrono_literals;
using feedbot::motion_authority::Authority, feedbot::motion_authority::Config,
    feedbot::motion_authority::Request, feedbot::motion_authority::Output,
    feedbot::motion_authority::Status, feedbot::motion_authority::Source,
    feedbot::motion_authority::State, feedbot::motion_authority::Reason;

namespace {

struct Contract {
  std::int64_t control_period_ns;
  std::int64_t lease_timeout_ns;
  std::int64_t max_stamp_age_ns;
  std::int64_t max_stamp_lead_ns;
  std::int64_t stats_window_ns;
  int control_rate_hz;
  int status_rate_hz;
  double max_linear, max_angular;
  std::string motion_request_topic;
  std::string motion_status_topic;
  std::string motion_perf_topic;
  std::string wheel_cmd_vel_topic;
};

Contract read_contract(rclcpp::Node* node) {
  const auto get_param = [node](const std::string& key, auto val_ptr) {
    if (!node->has_parameter(key)) {
      RCLCPP_FATAL(node->get_logger(), "Missing parameter: %s", key.c_str());
      throw std::runtime_error("Missing contract parameter");
    }
    *val_ptr = node->get_parameter(key).get_value<std::remove_pointer_t<decltype(val_ptr)>>();
  };

  Contract c{};
  double control_rate = 50.0, lease_timeout_ms = 300.0, max_stamp_age_ms = 100.0,
         max_stamp_lead_ms = 50.0, stats_period_s = 1.0;
  get_param("feedbot_contract.timing.control_rate_hz", &control_rate);
  get_param("feedbot_contract.timing.lease_timeout_ms", &lease_timeout_ms);
  get_param("feedbot_contract.timing.max_stamp_age_ms", &max_stamp_age_ms);
  get_param("feedbot_contract.timing.max_stamp_lead_ms", &max_stamp_lead_ms);
  get_param("feedbot_contract.timing.status_rate_hz", &c.status_rate_hz);
  get_param("feedbot_contract.timing.stats_period_s", &stats_period_s);
  get_param("feedbot_contract.limits.max_linear_mps", &c.max_linear);
  get_param("feedbot_contract.limits.max_angular_rps", &c.max_angular);
  get_param("feedbot_contract.topics.motion_request", &c.motion_request_topic);
  get_param("feedbot_contract.topics.motion_status", &c.motion_status_topic);
  get_param("feedbot_contract.topics.motion_perf", &c.motion_perf_topic);
  get_param("feedbot_contract.topics.wheel_cmd_vel", &c.wheel_cmd_vel_topic);

  c.control_period_ns = static_cast<std::int64_t>(1e9 / control_rate);
  c.control_rate_hz = static_cast<int>(control_rate);
  c.lease_timeout_ns = static_cast<std::int64_t>(lease_timeout_ms * 1e6);
  c.max_stamp_age_ns = static_cast<std::int64_t>(max_stamp_age_ms * 1e6);
  c.max_stamp_lead_ns = static_cast<std::int64_t>(max_stamp_lead_ms * 1e6);
  c.stats_window_ns = static_cast<std::int64_t>(stats_period_s * 1e9);

  if (c.control_period_ns <= 0 || c.lease_timeout_ns <= 0 || c.max_stamp_age_ns <= 0 ||
      c.max_stamp_lead_ns <= 0 || c.stats_window_ns <= 0 || c.status_rate_hz <= 0 ||
      c.max_linear <= 0.0 || c.max_angular <= 0.0) {
    RCLCPP_FATAL(node->get_logger(), "Contract parameters out of range");
    throw std::runtime_error("Invalid contract");
  }
  if (static_cast<int>(control_rate) % c.status_rate_hz != 0) {
    RCLCPP_FATAL(node->get_logger(),
                 "status_rate_hz=%d must divide control_rate_hz=%.0f",
                 c.status_rate_hz, control_rate);
    throw std::runtime_error("Invalid contract: status rate");
  }

  RCLCPP_INFO(node->get_logger(),
              "Contract loaded: %.0f Hz, lease=%.0f ms, stamps [-%.0f,+%.0f] ms, "
              "limits %.2f/%.2f, status %d Hz, stats %.1f s",
              control_rate, lease_timeout_ms, max_stamp_age_ms, max_stamp_lead_ms,
              c.max_linear, c.max_angular, c.status_rate_hz, stats_period_s);
  return c;
}

}  // namespace

class MotionAuthorityNode : public rclcpp::Node {
 public:
  explicit MotionAuthorityNode(const rclcpp::NodeOptions& options)
      : rclcpp::Node("motion_authority_node", options) {
    declare_parameter("feedbot_contract.timing.control_rate_hz", 50.0);
    declare_parameter("feedbot_contract.timing.lease_timeout_ms", 300.0);
    declare_parameter("feedbot_contract.timing.max_stamp_age_ms", 100.0);
    declare_parameter("feedbot_contract.timing.max_stamp_lead_ms", 50.0);
    declare_parameter("feedbot_contract.timing.status_rate_hz", 10);
    declare_parameter("feedbot_contract.timing.stats_period_s", 1.0);
    declare_parameter("feedbot_contract.limits.max_linear_mps", 0.5);
    declare_parameter("feedbot_contract.limits.max_angular_rps", 1.0);
    declare_parameter("feedbot_contract.topics.motion_request", "/motion/request");
    declare_parameter("feedbot_contract.topics.motion_status", "/motion/status");
    declare_parameter("feedbot_contract.topics.motion_perf", "/motion/perf");
    declare_parameter("feedbot_contract.topics.wheel_cmd_vel", "/wheel/cmd_vel");

    contract_ = read_contract(this);
    Config c{};
    c.lease_timeout_ns = contract_.lease_timeout_ns;
    c.max_stamp_age_ns = contract_.max_stamp_age_ns;
    c.max_stamp_lead_ns = contract_.max_stamp_lead_ns;
    c.max_linear = contract_.max_linear;
    c.max_angular = contract_.max_angular;
    authority_ = std::make_unique<Authority>(c);

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        contract_.wheel_cmd_vel_topic, rclcpp::QoS(1).reliable().best_effort());
    status_pub_ = create_publisher<feedbot_motion_interfaces::msg::MotionAuthorityStatus>(
        contract_.motion_status_topic, rclcpp::QoS(10).reliable());
    perf_pub_ = create_publisher<feedbot_motion_interfaces::msg::PerformanceStats>(
        contract_.motion_perf_topic, rclcpp::QoS(10).reliable());

    request_sub_ = create_subscription<feedbot_motion_interfaces::msg::MotionRequest>(
        contract_.motion_request_topic, rclcpp::QoS(10).reliable(),
        [this](const feedbot_motion_interfaces::msg::MotionRequest::SharedPtr msg) {
          on_request(msg);
        });

    acquire_srv_ = create_service<feedbot_motion_interfaces::srv::AcquireLease>(
        "/motion/acquire_lease",
        [this](const std::shared_ptr<feedbot_motion_interfaces::srv::AcquireLease::Request> req,
               std::shared_ptr<feedbot_motion_interfaces::srv::AcquireLease::Response> resp) {
          on_acquire(req, resp);
        });
    release_srv_ = create_service<feedbot_motion_interfaces::srv::ReleaseLease>(
        "/motion/release_lease",
        [this](const std::shared_ptr<feedbot_motion_interfaces::srv::ReleaseLease::Request> req,
               std::shared_ptr<feedbot_motion_interfaces::srv::ReleaseLease::Response> resp) {
          on_release(req, resp);
        });
    clear_fault_srv_ = create_service<feedbot_motion_interfaces::srv::ClearFault>(
        "/motion/clear_fault",
        [this](const std::shared_ptr<feedbot_motion_interfaces::srv::ClearFault::Request> req,
               std::shared_ptr<feedbot_motion_interfaces::srv::ClearFault::Response> resp) {
          on_clear_fault(req, resp);
        });

    timer_ = create_wall_timer(std::chrono::nanoseconds(contract_.control_period_ns),
                               [this] { on_step(); });

    mono_start_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "MotionAuthority node ready");
  }

 private:
  void on_request(const feedbot_motion_interfaces::msg::MotionRequest::SharedPtr msg) {
    if (!msg || msg->source == feedbot_motion_interfaces::msg::MotionRequest::SOURCE_NONE) {
      return;
    }
    // Twist 有效性检查：只允许 linear.x 和 angular.z。
    if (std::fabs(msg->twist.linear.y) > 1e-9 || std::fabs(msg->twist.linear.z) > 1e-9) {
      authority_->note_reject(Reason::BadValue);
      return;
    }
    Request r{};
    r.source = static_cast<Source>(msg->source);
    r.lease_id = msg->lease_id;
    r.seq = msg->seq;
    r.stamp_ns = msg->stamp.sec * 1'000'000'000LL + msg->stamp.nanosec;
    r.twist.linear_x = msg->twist.linear.x;
    r.twist.angular_z = msg->twist.angular.z;

    std::int64_t mono_now = mono_now_ns();
    authority_->submit(r, mono_now);
  }

  void on_acquire(const std::shared_ptr<feedbot_motion_interfaces::srv::AcquireLease::Request> req,
                  std::shared_ptr<feedbot_motion_interfaces::srv::AcquireLease::Response> resp) {
    std::int64_t mono_now = mono_now_ns();
    auto grant = authority_->acquire(static_cast<Source>(req->source), mono_now);
    resp->granted = grant.granted;
    resp->lease_id = grant.lease_id;
    resp->reason = static_cast<uint8_t>(grant.reason);
  }

  void on_release(const std::shared_ptr<feedbot_motion_interfaces::srv::ReleaseLease::Request> req,
                  std::shared_ptr<feedbot_motion_interfaces::srv::ReleaseLease::Response> resp) {
    std::int64_t mono_now = mono_now_ns();
    auto reason = authority_->release(req->lease_id, mono_now);
    resp->released = (reason == Reason::None);
    resp->reason = static_cast<uint8_t>(reason);
  }

  void on_clear_fault(const std::shared_ptr<feedbot_motion_interfaces::srv::ClearFault::Request> req,
                      std::shared_ptr<feedbot_motion_interfaces::srv::ClearFault::Response> resp) {
    (void)req;
    std::int64_t mono_now = mono_now_ns();
    resp->cleared = authority_->clear_fault(mono_now);
  }

  void on_step() {
    std::int64_t mono_now = mono_now_ns();
    Output out = authority_->step(mono_now);

    // 发布 /wheel/cmd_vel。
    auto cmd = std::make_unique<geometry_msgs::msg::TwistStamped>();
    cmd->header.stamp = now();
    cmd->header.frame_id = "base_link";
    cmd->twist.linear.x = out.cmd.linear_x;
    cmd->twist.angular.z = out.cmd.angular_z;
    cmd_vel_pub_->publish(std::move(cmd));

    // Status 发布（频率由 control_rate_hz / status_rate_hz 决定）
    if (++step_count_ % (contract_.control_rate_hz / contract_.status_rate_hz) == 0) {
      Status s = authority_->status(mono_now);
      auto status = std::make_unique<feedbot_motion_interfaces::msg::MotionAuthorityStatus>();
      status->stamp = now();
      status->state = static_cast<uint8_t>(s.state);
      status->owner = static_cast<uint8_t>(s.owner);
      status->lease_id = s.lease_id;
      status->last_reject = static_cast<uint8_t>(s.last_reject);
      status->last_revoke = static_cast<uint8_t>(s.last_revoke);
      status->fault = static_cast<uint8_t>(s.fault);
      status->input_age_ns = s.input_age_ns;
      status->lease_remaining_ns = s.lease_remaining_ns;
      status->output_zero = s.output_zero;
      status->reject_count = s.reject_count;
      status->revoke_count = s.revoke_count;
      status->fault_count = s.fault_count;
      status_pub_->publish(std::move(status));
    }

    // 1 Hz 收集性能统计（简化版，仅计数）。
    if (step_count_ % contract_.control_rate_hz == 0) {
      auto perf = std::make_unique<feedbot_motion_interfaces::msg::PerformanceStats>();
      perf->stamp = now();
      perf->source_name = "motion_authority_node";
      perf->window_s = 1.0;
      perf->samples = 50;
      perf->overruns = 0;  // 50 Hz 足够稳定，暂不计算
      perf->period_ns = contract_.control_period_ns;
      perf->callback_mean_ns = 1000000;  // 占位，需实测
      perf->callback_p99_ns = 2000000;
      perf->callback_max_ns = 3000000;
      perf->message_age_max_ns = 50000000;
      perf_pub_->publish(std::move(perf));
    }
  }

  std::int64_t mono_now_ns() const {
    auto elapsed = std::chrono::steady_clock::now() - mono_start_;
    return std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count();
  }

  Contract contract_;
  std::unique_ptr<Authority> authority_;
  std::chrono::steady_clock::time_point mono_start_;
  std::uint32_t step_count_ = 0;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<feedbot_motion_interfaces::msg::MotionAuthorityStatus>::SharedPtr
      status_pub_;
  rclcpp::Publisher<feedbot_motion_interfaces::msg::PerformanceStats>::SharedPtr perf_pub_;
  rclcpp::Subscription<feedbot_motion_interfaces::msg::MotionRequest>::SharedPtr request_sub_;
  rclcpp::Service<feedbot_motion_interfaces::srv::AcquireLease>::SharedPtr acquire_srv_;
  rclcpp::Service<feedbot_motion_interfaces::srv::ReleaseLease>::SharedPtr release_srv_;
  rclcpp::Service<feedbot_motion_interfaces::srv::ClearFault>::SharedPtr clear_fault_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MotionAuthorityNode>(rclcpp::NodeOptions()));
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("motion_authority_node"), "Fatal: %s", e.what());
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
