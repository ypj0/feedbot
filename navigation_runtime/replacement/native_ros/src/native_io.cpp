#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "feedbot_native/msg/motor_command.hpp"
#include "feedbot_native/msg/native_state.hpp"
#include "chassis/serial_port.hpp"
#include "chassis/transport.hpp"
#include "chassis/odometry.hpp"
#include "chassis/stationarity.hpp"
#include "imu/wit_decoder.hpp"
#include <atomic>
#include <mutex>
#include <random>
#include <optional>
#include <iostream>
#include <array>
#include <vector>
using namespace feedbot_chassis;
using Command=feedbot_native::msg::MotorCommand;
using Status=feedbot_native::msg::NativeState;
using Trigger=std_srvs::srv::Trigger;
class NativeIO:public rclcpp::Node {
 struct Receipt {uint64_t sequence=0;double received=0,applied=0,expires=0;bool accepted=false;};
 struct Queued {Command command;double received;};
 struct Snapshot {bool armed=false,arm_ready=false;std::string arm_reason="starting";Feedback f{};Pose2 pose{};double linear_velocity=0,angular_velocity=0;feedbot_imu::Snapshot imu{};
  uint64_t sequence=0,epoch=0;double received=0;bool fault=true,odom_valid=false;std::string reason="starting";std::vector<Receipt> fault_commands;double fault_at=0;};
 std::string reply_reason_;
 std::mutex in_mutex_,out_mutex_;std::optional<Queued> pending_;Snapshot published_;
 std::atomic<bool> quit_{false},stop_{false};std::thread worker_;
 bool allow_motion_=false;std::string chassis_path_,imu_path_;bool fixture_=false;
 std::atomic<int> request_{0},reply_{0};std::atomic<uint64_t> epoch_public_{0};
 rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
 rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
 rclcpp::Publisher<Status>::SharedPtr status_pub_;
 rclcpp::Subscription<Command>::SharedPtr command_sub_;
 rclcpp::Service<Trigger>::SharedPtr arm_,reset_,stop_srv_;
 rclcpp::TimerBase::SharedPtr timer_;
 uint64_t previous_odom_=0,publish_cycles_=0;double previous_imu_=0;bool fault_reported_=false;
 struct Reactor {
  Snapshot s;WheelOdometry odom;uint64_t seq=0;double last=0;
  explicit Reactor(Calibration c):odom(c){}
 };
 static void feedback_received(const Feedback& f,double,void* context){
  auto& r=*static_cast<Reactor*>(context);double t=steady_seconds();++r.seq;
  if(r.seq==1)r.odom.begin(1,r.seq,t);
  else r.odom.update(1,r.seq,t,f);
  r.s.f=f;r.s.received=t;r.s.sequence=r.seq;r.s.pose=r.odom.pose();r.s.odom_valid=r.odom.velocity_valid();
  r.s.linear_velocity=r.odom.linear_velocity();r.s.angular_velocity=r.odom.angular_velocity();
  r.last=t;
 }
 void publish_snapshot(){
  Snapshot s;{std::lock_guard<std::mutex> lock(out_mutex_);s=published_;}
  if(s.fault_at&&!fault_reported_){
   fault_reported_=true;
   RCLCPP_ERROR(get_logger(),"actuator_fault steady=%.9f reason=%s",s.fault_at,s.reason.c_str());
   for(const auto& c:s.fault_commands)RCLCPP_ERROR(get_logger(),"actuator_command sequence=%lu received=%.9f applied=%.9f expires=%.9f accepted=%d",c.sequence,c.received,c.applied,c.expires,c.accepted);
  }
  double now=steady_seconds();bool fresh=s.sequence&&now>=s.received&&now-s.received<.2;
  Status st;st.epoch=s.epoch;st.sequence=s.sequence;st.received_steady_ns=uint64_t(s.received*1e9);
  st.motion_enabled=allow_motion_;st.faulted=s.fault||!fresh;st.odometry_valid=s.odom_valid&&fresh;
  double imu_received=*std::max_element(s.imu.received.begin(),s.imu.received.end());
  bool imu_fresh=s.imu.ready&&now>=imu_received&&now-*std::min_element(s.imu.received.begin(),s.imu.received.end())<.1;
  st.imu_ready=imu_fresh;st.emergency=s.f.emergency;st.motor_free=s.f.motor_free;st.charge=s.f.charge;
  st.armed=s.armed;st.arm_ready=s.arm_ready&&fresh&&imu_fresh;st.arm_reason=st.arm_ready?"":(!fresh||!imu_fresh?"sensor feedback stale":s.arm_reason);
  st.battery=s.f.power;st.firmware=s.f.version;st.linear_raw=s.f.v_raw;st.angular_raw=s.f.w_raw;st.reason=s.reason;
  if(++publish_cycles_%4==0)status_pub_->publish(st);
  auto stamp=get_clock()->now()-rclcpp::Duration::from_seconds(now-s.received);
  if(fresh&&s.odom_valid&&s.sequence!=previous_odom_){
   previous_odom_=s.sequence;
   nav_msgs::msg::Odometry m;m.header.stamp=stamp;m.header.frame_id="wheel_odom";m.child_frame_id="base_footprint";
   m.pose.pose.position.x=s.pose.x;m.pose.pose.position.y=s.pose.y;
   m.pose.pose.orientation.z=std::sin(s.pose.yaw/2);m.pose.pose.orientation.w=std::cos(s.pose.yaw/2);
   m.twist.twist.linear.x=s.linear_velocity;m.twist.twist.angular.z=s.angular_velocity;
   // Provisional covariance; physical calibration is not inferred from these values.
   for(int i=0;i<6;i++){m.pose.covariance[i*7]=i==0||i==1||i==5?.05:1e6;m.twist.covariance[i*7]=i==0||i==5?.05:1e6;}
   odom_pub_->publish(m);
  }
  if(imu_fresh&&imu_received!=previous_imu_){
   previous_imu_=imu_received;
   sensor_msgs::msg::Imu m;m.header.stamp=get_clock()->now()-rclcpp::Duration::from_seconds(now-imu_received);m.header.frame_id="imu_link";
   auto q=s.imu.quaternion;m.orientation.x=q[0];m.orientation.y=q[1];m.orientation.z=q[2];m.orientation.w=q[3];
   auto a=s.imu.acceleration,g=s.imu.angular_velocity;
   m.linear_acceleration.x=a[0];m.linear_acceleration.y=a[1];m.linear_acceleration.z=a[2];
   m.angular_velocity.x=g[0];m.angular_velocity.y=g[1];m.angular_velocity.z=g[2];
   for(int i=0;i<3;i++){m.orientation_covariance[i*4]=.02;m.angular_velocity_covariance[i*4]=.02;m.linear_acceleration_covariance[i*4]=.1;}
   imu_pub_->publish(m);
  }
 }
 void service(int code,std::shared_ptr<Trigger::Response> response){
  if(code==3){stop_=true;response->success=true;response->message="stop latched; acknowledgement is not a measured physical stop";return;}
  if(code==1&&!allow_motion_){response->message="motion commissioning not enabled";return;}
  int empty=0;if(!request_.compare_exchange_strong(empty,-1)){response->message="request busy";return;}
  reply_=0;request_=code;
  auto end=steady_seconds()+.25;
  while(!reply_&&steady_seconds()<end&&!quit_)std::this_thread::sleep_for(std::chrono::milliseconds(1));
  response->success=reply_==1;
  {std::lock_guard<std::mutex> lock(out_mutex_);
   response->message=response->success?std::to_string(epoch_public_.load()):(reply_?reply_reason_:"reactor request timeout");}
  if(!response->success)RCLCPP_WARN(get_logger(),"actuator_request code=%d refused=%s",code,response->message.c_str());
  if(!reply_)stop_=true; // A timed-out request must never cause delayed motion.
 }
 void run(){
  Reactor r({});feedbot_imu::Decoder imu;Stationarity stationary;
  try{
   SerialPort chassis(chassis_path_,fixture_),imu_port(imu_path_,fixture_);chassis.handshake();
   Transport transport(chassis.fd());transport.on_feedback(feedback_received,&r);
   std::random_device rng;uint64_t epoch=(uint64_t(rng())<<32)|rng();epoch&=UINT64_MAX>>1;epoch+=1;
   double previous=steady_seconds(),output_blocked_since=0;
   std::array<Receipt,128> history{};size_t history_count=0;
   while(!quit_){
    double now=steady_seconds();double reactor_gap=now-previous;if(transport.armed()&&reactor_gap>.04)transport.stop("serial reactor scheduling deadline exceeded");previous=now;
    if(stop_)transport.cancel();
    int queued=0;
    require_serial(ioctl(chassis.fd(),TIOCOUTQ,&queued)==0,"cannot inspect chassis TX queue");
    if(queued){if(!output_blocked_since)output_blocked_since=now;require_serial(queued<=80&&now-output_blocked_since<.04,"chassis TX backlog");}
    else output_blocked_since=0;
    require_serial(ioctl(chassis.fd(),FIONREAD,&queued)==0&&queued<=400,"chassis RX backlog");
    require_serial(ioctl(imu_port.fd(),FIONREAD,&queued)==0,"cannot inspect IMU RX queue");
    if(queued>330){
     RCLCPP_WARN(get_logger(),"imu_rx_resync steady=%.9f queued=%d reactor_gap=%.6f armed=%d",
       now,queued,reactor_gap,transport.armed());
     if(transport.armed())transport.stop("IMU RX backlog");
     require_serial(tcflush(imu_port.fd(),TCIFLUSH)==0,"cannot flush IMU RX backlog");
     imu=feedbot_imu::Decoder{};
    }
    uint8_t bytes[512];auto n=read(imu_port.fd(),bytes,sizeof(bytes));
    if(n>0)for(ssize_t i=0;i<n;i++)imu.push(bytes[i],now);
    else require_serial(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK||errno==EINTR),"IMU disconnected");
    r.s.imu=imu.snapshot(now,.1,.05);
    double imu_at=*std::min_element(r.s.imu.received.begin(),r.s.imu.received.end());
    stationary.observe(std::min(r.s.received,imu_at),r.s.linear_velocity,r.s.angular_velocity,r.s.imu.angular_velocity[2],
      r.s.imu.ready&&r.s.odom_valid&&now-r.s.received<.1);
    auto arm_blocker=[&]()->std::string{
      if(!allow_motion_)return "motion commissioning not enabled";
      if(!r.s.imu.ready||!r.s.odom_valid)return "IMU or encoder feedback unavailable";
      return transport.arm_blocker(now,stationary.ready(now));
    };
    int req=request_.load();
    if(req>0){
     bool ok=false;
     if(req==2){ok=transport.reset(now,stationary.ready(now));if(ok)stop_=false;}
     else if(req==1&&allow_motion_&&r.s.imu.ready&&r.s.odom_valid){ok=transport.arm(++epoch,now,stationary.ready(now));if(ok)stop_=false;}
     {std::lock_guard<std::mutex> lock(out_mutex_);
      reply_reason_=ok?"":(req==1?arm_blocker():"reset requires emergency and measured standstill");}
     r.s.epoch=epoch;epoch_public_=epoch;request_=0;reply_=ok?1:2;
    }
    {std::unique_lock<std::mutex> lock(in_mutex_,std::try_to_lock);
     if(lock&&pending_){auto queued_command=*pending_;auto cmd=queued_command.command;pending_.reset();
      bool accepted=false;
      if(allow_motion_&&!stop_&&r.s.imu.ready&&r.s.odom_valid)
       accepted=transport.command(cmd.epoch,cmd.sequence,cmd.linear,cmd.angular,cmd.expires_steady_ns*1e-9,now);
      history[history_count++%history.size()]={cmd.sequence,queued_command.received,now,cmd.expires_steady_ns*1e-9,accepted};
     }
    }
    // Missing initial samples are startup readiness, not a motion fault.
    // Once armed, the same missing feedback immediately trips the lease.
    if(transport.armed()&&(!r.s.imu.ready||!r.s.odom_valid))transport.stop("IMU or encoder feedback invalid while armed");
    transport.step(now);require_serial(!transport.failed(),"chassis transport failed");
    if(!allow_motion_&&r.s.sequence)require_serial(r.s.f.emergency==1&&r.s.f.v_raw==0&&r.s.f.w_raw==0,"maintenance condition changed");
    if(transport.faulted()&&!r.s.fault_at){
     r.s.fault_at=now;
     for(size_t i=history_count>history.size()?history_count-history.size():0;i<history_count;++i)r.s.fault_commands.push_back(history[i%history.size()]);
    }
    r.s.armed=transport.armed();r.s.arm_reason=arm_blocker();r.s.arm_ready=r.s.arm_reason.empty();
    r.s.fault=transport.faulted();r.s.reason=transport.faulted()?transport.fault_reason():allow_motion_?"commissioning":"maintenance: output restricted to zero";
    {std::unique_lock<std::mutex> lock(out_mutex_,std::try_to_lock);if(lock)published_=r.s;}
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
   }
   transport.stop();for(unsigned i=0;i<25&&!transport.failed();i++){transport.step(steady_seconds());std::this_thread::sleep_for(std::chrono::milliseconds(2));}
  }catch(const std::exception& e){r.s.fault=true;r.s.odom_valid=false;r.s.imu.ready=false;r.s.reason=e.what();}
  std::lock_guard<std::mutex> lock(out_mutex_);published_=r.s;
 }
public:
 NativeIO():Node("native_io"){
  rcl_interfaces::msg::ParameterDescriptor immutable;immutable.read_only=true;
  fixture_=declare_parameter("fixture",false,immutable);
  allow_motion_=declare_parameter("allow_motion",false,immutable);
  chassis_path_=declare_parameter("chassis_device",std::string("/dev/rikibase"),immutable);
  imu_path_=declare_parameter("imu_device",std::string("/dev/rikiimu"),immutable);
  if(fixture_&&std::string(get_namespace()).find("/fixture")!=0)throw std::runtime_error("fixtures require /fixture namespace");
  odom_pub_=create_publisher<nav_msgs::msg::Odometry>("~/wheel_odom",rclcpp::SensorDataQoS());
  imu_pub_=create_publisher<sensor_msgs::msg::Imu>("~/imu",rclcpp::SensorDataQoS());
  status_pub_=create_publisher<Status>("~/state",10);
  command_sub_=create_subscription<Command>("~/command",rclcpp::QoS(1).best_effort(),[this](Command::SharedPtr m){std::lock_guard<std::mutex> lock(in_mutex_);pending_=Queued{*m,steady_seconds()};});
  arm_=create_service<Trigger>("~/arm",[this](const std::shared_ptr<Trigger::Request>,std::shared_ptr<Trigger::Response> r){service(1,r);});
  reset_=create_service<Trigger>("~/reset",[this](const std::shared_ptr<Trigger::Request>,std::shared_ptr<Trigger::Response> r){service(2,r);});
  stop_srv_=create_service<Trigger>("~/stop",[this](const std::shared_ptr<Trigger::Request>,std::shared_ptr<Trigger::Response> r){service(3,r);});
  timer_=create_wall_timer(std::chrono::milliseconds(5),[this]{publish_snapshot();});
  worker_=std::thread([this]{run();});
 }
 ~NativeIO(){quit_=true;if(worker_.joinable())worker_.join();}
};
int main(int argc,char** argv){rclcpp::init(argc,argv);try{auto node=std::make_shared<NativeIO>();rclcpp::spin(node);node.reset();rclcpp::shutdown();return 0;}catch(const std::exception& e){std::cerr<<e.what()<<std::endl;rclcpp::shutdown();return 1;}}
