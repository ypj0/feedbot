#pragma once
#include "protocol.hpp"
namespace feedbot_chassis {
struct WheelSample {double left_m,right_m,v_mps,w_radps;};
struct Calibration {
 double left_scale=1,right_scale=1,track_m=.455;
 void verify()const{
  if(!std::isfinite(left_scale)||!std::isfinite(right_scale)||!std::isfinite(track_m)||
     left_scale<=0||right_scale<=0||track_m<=0)throw std::invalid_argument("invalid wheel calibration");
 }
};
// Compatibility conversion from the observed v441 protocol. The MCU already
// differences its counters and reverses the right motor's sign before packing.
// These are per-response increments; never difference two response values again.
// Scaling reproduces read_odom's integer truncation, rather than inventing pulses.
// Physical distance/direction calibration remains a commissioning requirement.
inline WheelSample wheel_sample(const Feedback& f,const Calibration& c){
 c.verify();if(f.version!=441||f.initialized!=1)throw std::invalid_argument("unsupported feedback version/state");
 auto delta=[](int raw,double scale){
  double units=((double(raw)*1000.)/scale)/1000.;
  if(raw< -32768||raw>32767||!std::isfinite(units)||units< -32768||units>32767)
   throw std::invalid_argument("wheel increment out of range");
  return double(int(units))/100000.;
 };
 if(f.v_raw< -32768||f.v_raw>32767||f.w_raw< -32768||f.w_raw>32767)
  throw std::invalid_argument("raw speed out of range");
 return {delta(f.left_encoder,c.left_scale),delta(f.right_encoder,c.right_scale),
         double(f.v_raw)/1000.,-double(f.w_raw)/1000.};
}
struct Pose2 {double x=0,y=0,yaw=0;};
// Wheel-only odometry, not localization or IMU fusion. This uses the SE(2)
// constant-curvature increment, not the factory's previous-heading Euler step.
// Neither this class nor wheel_sample publishes TF or authorizes motion.
class WheelOdometry {
 Calibration c_;Pose2 pose_{};uint64_t epoch_=0,sequence_=0;double time_=0;
 bool started_=false,valid_=false;
 struct VelocityPoint {double time,distance,angle;};
 // Bounded cumulative-increment window: pose and twist use the same wheel evidence.
 std::array<VelocityPoint,64> window_{};unsigned head_=0,count_=0;
 double distance_=0,angle_=0,v_=0,w_=0;bool velocity_valid_=false;
 void velocity(double time){
  if(count_==window_.size()){valid_=velocity_valid_=false;return;}
  window_[(head_+count_)%window_.size()]={time,distance_,angle_};++count_;
  while(count_>2 && time-window_[(head_+1)%window_.size()].time>=.1){head_=(head_+1)%window_.size();--count_;}
  const auto& first=window_[head_];double span=time-first.time;
  velocity_valid_=span>=.1-1e-9;
  v_=velocity_valid_?(distance_-first.distance)/span:0.;
  w_=velocity_valid_?(angle_-first.angle)/span:0.;
 }
public:
 explicit WheelOdometry(Calibration c={}):c_(c){c_.verify();}
 void begin(uint64_t epoch,uint64_t sequence,double time){
  if(epoch==0||epoch<=epoch_||!std::isfinite(time)||time<0)throw std::invalid_argument("invalid odometry epoch");
  epoch_=epoch;sequence_=sequence;time_=time;started_=valid_=true;pose_={};
  head_=count_=0;distance_=angle_=v_=w_=0.;velocity_valid_=false;velocity(time);
 }
 bool valid()const{return valid_;}
 bool velocity_valid()const{return valid_&&velocity_valid_;}
 double linear_velocity()const{return v_;}
 double angular_velocity()const{return w_;}
 Pose2 pose()const{return pose_;}
 bool update(uint64_t epoch,uint64_t sequence,double time,const Feedback& f){
  if(!started_||!valid_)return false;
  if(epoch!=epoch_||sequence_==UINT64_MAX||sequence!=sequence_+1||!std::isfinite(time)||
     time<=time_||time-time_>.2){valid_=false;return false;}
  WheelSample s;
  try{s=wheel_sample(f,c_);}catch(...){valid_=false;return false;}
  double distance=(s.left_m+s.right_m)*.5,angle=(s.right_m-s.left_m)/c_.track_m;
  double half=angle*.5;
  double sinc=std::abs(half)<1e-6?1-half*half/6:std::sin(half)/half;
  pose_.x+=distance*sinc*std::cos(pose_.yaw+half);
  pose_.y+=distance*sinc*std::sin(pose_.yaw+half);
  pose_.yaw=std::remainder(pose_.yaw+angle,2*std::acos(-1.));
  distance_+=distance;angle_+=angle;velocity(time);
  sequence_=sequence;time_=time;return valid_;
 }
};
}
