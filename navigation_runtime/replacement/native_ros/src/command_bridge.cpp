#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "feedbot_native/msg/motor_command.hpp"
#include "feedbot_native/msg/native_state.hpp"
#include "feedbot_native/msg/task_lease.hpp"
#include <chrono>
#include <cmath>
using Trigger=std_srvs::srv::Trigger;
using NativeState=feedbot_native::msg::NativeState;
using MotorCommand=feedbot_native::msg::MotorCommand;
using TaskLease=feedbot_native::msg::TaskLease;
double steady(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
class Bridge:public rclcpp::Node {
 NativeState state_;double state_at_=0,scan_at_=0,task_deadline_=0,task_started_=0,command_at_=0;
 uint64_t task_=0,largest_task_=0,epoch_=0,sequence_=0,generation_=0;
 bool raw_stop_=false; double raw_stamp_=0, resume_stamp_=0;
 bool arming_=false,blocked_=true;std::string reason_="no task";
 geometry_msgs::msg::TwistStamped latest_;
 rclcpp::Client<Trigger>::SharedPtr arm_,stop_;
 rclcpp::Publisher<MotorCommand>::SharedPtr motor_;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
 rclcpp::Subscription<NativeState>::SharedPtr state_sub_;
 rclcpp::Subscription<TaskLease>::SharedPtr task_sub_;
 rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
 rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr command_sub_,raw_sub_;
 rclcpp::TimerBase::SharedPtr timer_, source_deadline_timer_;
 void halt(const std::string& why){
  if(epoch_||arming_){
   const double t=steady();
   RCLCPP_WARN(get_logger(),"halt=%s task=%lu state_age=%.6f scan_age=%.6f command_age=%.6f lease_remaining=%.6f",why.c_str(),task_,t-state_at_,t-scan_at_,t-command_at_,task_deadline_-t);
  }
  if(source_deadline_timer_)source_deadline_timer_->cancel();
  bool was_live=epoch_||arming_;++generation_;epoch_=0;blocked_=true;reason_=why;
  if(was_live&&stop_->service_is_ready())stop_->async_send_request(std::make_shared<Trigger::Request>());
 }
 bool healthy(double t){return state_.motion_enabled&&!state_.faulted&&state_.odometry_valid&&state_.imu_ready&&
   state_.emergency==0&&state_.motor_free==0&&state_.charge==0&&t-state_at_<.1&&t-scan_at_<.2;}
 // Source expiry is scheduled independently of the periodic output tick.
 // Match the configured smoother lifetime and native maximum lease; keep
 // a separate 20 ms delivery reserve within that absolute source deadline.
 static constexpr double delivery_budget_=.02;
 double source_lifetime_=.15;
 void schedule_source_deadline(){
  if(source_deadline_timer_)source_deadline_timer_->cancel();
  const double source=command_at_;
  auto delay=std::chrono::nanoseconds(std::max<int64_t>(1,int64_t((source+source_lifetime_-delivery_budget_-steady())*1e9)));
  source_deadline_timer_=create_wall_timer(delay,[this,source]{
   if(command_at_!=source)return;
   source_deadline_timer_->cancel();
   const double t=steady();
   if(epoch_&&task_&&!blocked_&&healthy(t)){
    publish_stop(t);reason_=raw_stop_||stopped_output()?"active":"waiting for checked velocity";
   }
  });
 }
 void output(double t){
  if(!epoch_||!task_||blocked_||!healthy(t))return;
  if(raw_stop_||stopped_output()||t-command_at_>=source_lifetime_-delivery_budget_){
   publish_stop(t);reason_=raw_stop_||stopped_output()?"active":"waiting for checked velocity";return;
  }
  MotorCommand cmd;cmd.epoch=epoch_;cmd.sequence=++sequence_;
  cmd.expires_steady_ns=uint64_t((command_at_+source_lifetime_)*1e9);
  cmd.linear=latest_.twist.linear.x;cmd.angular=latest_.twist.angular.z;motor_->publish(cmd);reason_="active";
 }
 bool stopped_output()const{return latest_.twist.linear.x==0.&&latest_.twist.angular.z==0.;}
 void publish_stop(double t){
  // This is a new fail-safe stop decision, not a renewal of a stale moving
  // command. Task ownership and current sensor interlocks still govern it.
  MotorCommand cmd;cmd.epoch=epoch_;cmd.sequence=++sequence_;
  cmd.expires_steady_ns=uint64_t((t+.1)*1e9);motor_->publish(cmd);
 }
 void tick(){
  double t=steady();
  if(task_&&t>=task_deadline_){halt("task lease expired");task_=0;}
  if(epoch_&&!healthy(t)){halt("feedback or controller stale");}
  output(t);
  std_msgs::msg::String msg;msg.data=reason_;status_->publish(msg);
 }
 void command(geometry_msgs::msg::TwistStamped::SharedPtr m){
  double t=steady(),age=(now()-rclcpp::Time(m->header.stamp)).seconds();
  if(!task_||blocked_||t>=task_deadline_)return;
  if(raw_stop_ || rclcpp::Time(m->header.stamp).seconds()<resume_stamp_)return;
  if(std::isfinite(age)&&age>=source_lifetime_)return; // Late input cannot renew authority.
  if(!healthy(t)){halt("native feedback or scan unavailable");return;}
  auto v=m->twist.linear.x,w=m->twist.angular.z;
  if(m->header.frame_id!="base_footprint"||!std::isfinite(age)||age<0||age>=source_lifetime_||
     !std::isfinite(v)||!std::isfinite(w)||std::abs(v)>.5||std::abs(w)>1.||
     m->twist.linear.y!=0||m->twist.linear.z!=0||m->twist.angular.x!=0||m->twist.angular.y!=0){halt("invalid controller output");return;}
  if(t-age<task_started_||rclcpp::Time(m->header.stamp)<rclcpp::Time(latest_.header.stamp))return;
  // Smoother outputs share the raw source stamp but carry newer profiled
  // velocities. Accept ordered DDS updates without renewing source authority.
  const bool new_source=rclcpp::Time(m->header.stamp)>rclcpp::Time(latest_.header.stamp);
  latest_=*m;
  if(new_source){command_at_=t-age;schedule_source_deadline();}
  if(!epoch_){
   if(arming_)return;
   if(!arm_->service_is_ready()){halt("actuator arm service unavailable");return;}
   arming_=true;auto generation=generation_;
   arm_->async_send_request(std::make_shared<Trigger::Request>(),[this,generation](rclcpp::Client<Trigger>::SharedFuture f){
    arming_=false;
    if(generation!=generation_){if(stop_->service_is_ready())stop_->async_send_request(std::make_shared<Trigger::Request>());return;}
    auto r=f.get();
    if(!r->success){halt("actuator refused arm: "+r->message);return;}
    try{epoch_=std::stoull(r->message);}catch(...){halt("invalid actuator epoch");return;}
    sequence_=0;reason_="active";
   });
   return;
  }
  output(t);
 }
public:
 Bridge():Node("command_bridge"){
  rcl_interfaces::msg::ParameterDescriptor immutable;immutable.read_only=true;
  source_lifetime_=declare_parameter("source_lifetime_s",.15,immutable);
  if(!std::isfinite(source_lifetime_) || source_lifetime_<=delivery_budget_ || source_lifetime_>.15)
   throw std::invalid_argument("command source lifetime must fit the native 150 ms lease");
  std::string native=declare_parameter("native_prefix",std::string("/native/io"));
  arm_=create_client<Trigger>(native+"/arm");stop_=create_client<Trigger>(native+"/stop");
  motor_=create_publisher<MotorCommand>(native+"/command",rclcpp::QoS(1).best_effort());
  status_=create_publisher<std_msgs::msg::String>("~/status",10);
  state_sub_=create_subscription<NativeState>(native+"/state",rclcpp::QoS(1),[this](NativeState::SharedPtr s){
   double t=steady(),received=s->received_steady_ns*1e-9;
   // All native processes share the host monotonic clock. DDS delivery cannot
   // refresh a sample which was already old at the serial owner.
   if(received<=0||received>t||t-received>=.1)return;
   if(state_.received_steady_ns>s->received_steady_ns)return;
   state_=*s;state_at_=received;
  });
  scan_sub_=create_subscription<sensor_msgs::msg::LaserScan>("scan",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::LaserScan::SharedPtr m){
   double age=(now()-rclcpp::Time(m->header.stamp)).seconds();if(!m->ranges.empty()&&age>=0&&age<.2)scan_at_=steady()-age;
  });
  task_sub_=create_subscription<TaskLease>("task_lease",rclcpp::QoS(1).reliable(),[this](TaskLease::SharedPtr m){
   double t=steady(),end=m->expires_steady_ns*1e-9;
   if(!m->task){halt("task cancelled");task_=0;return;}
   if(end<=t||end-t>.2)return;
   if(m->task!=task_){
    if(m->task<=largest_task_)return;
    if(task_||arming_||epoch_)return; // Cancel/finish the current owner before replacement.
    halt("task changed");task_=largest_task_=m->task;task_started_=t;latest_=geometry_msgs::msg::TwistStamped();raw_stop_=false;raw_stamp_=0;resume_stamp_=0;blocked_=false;reason_="waiting for controller";
   }
   task_deadline_=end;
  });
  // Upstream stop intent is a veto, never an alternative motion source.
  // The smoother may still emit deceleration samples after FollowPath halts.
  // Hold zero until a newer controller intent AND its checked downstream
  // output arrive. Never renew a nonzero command during a planned wait.
  raw_sub_=create_subscription<geometry_msgs::msg::TwistStamped>("cmd_vel_raw",rclcpp::QoS(1).reliable(),[this](geometry_msgs::msg::TwistStamped::SharedPtr m){
   const double t=steady(),stamp=rclcpp::Time(m->header.stamp).seconds(),age=now().seconds()-stamp;
   if(!task_||blocked_||t>=task_deadline_||age<0||age>=source_lifetime_||t-age<task_started_||stamp<=raw_stamp_)return;
   const auto &v=m->twist;
   if(m->header.frame_id!="base_footprint"||!std::isfinite(v.linear.x)||!std::isfinite(v.angular.z)||
      v.linear.y!=0||v.linear.z!=0||v.angular.x!=0||v.angular.y!=0)return;
   raw_stamp_=stamp;
   if(v.linear.x==0.&&v.angular.z==0.){
    raw_stop_=true;latest_.twist=geometry_msgs::msg::Twist();
    if(epoch_&&healthy(t))publish_stop(t);
   }else if(raw_stop_){raw_stop_=false;resume_stamp_=stamp;}
  });
  command_sub_=create_subscription<geometry_msgs::msg::TwistStamped>("cmd_vel_safe",rclcpp::QoS(1).reliable(),[this](geometry_msgs::msg::TwistStamped::SharedPtr m){command(m);});
  timer_=create_wall_timer(std::chrono::milliseconds(20),[this]{tick();});
 }
};
int main(int argc,char** argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<Bridge>());rclcpp::shutdown();}
