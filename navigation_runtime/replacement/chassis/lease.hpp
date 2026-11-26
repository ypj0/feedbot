#pragma once
#include "protocol.hpp"
#include <cstdint>
namespace feedbot_chassis {
// Pure state machine. No UART/ROS or hardware authorization is provided here.
// Caller advances one monotonic clock and serializes events; no read can block tick().
class Lease {
 const char* reason_="";
 Profile profile_;Feedback status_{};bool seen_=false,armed_=false,fault_=false;
 uint64_t epoch_=0,sequence_=0;double observed_=0,deadline_=0,v_=0,w_=0,last_now_=0;
 bool time_ok(double now){if(!std::isfinite(now)||now<last_now_){trip("non-monotonic lease clock");return false;}last_now_=now;return true;}
 bool healthy(double now)const{return seen_&&now-observed_<.2&&status_.version==441&&status_.initialized==1&&status_.emergency==0&&status_.motor_free==0&&status_.charge==0&&status_.mode==2&&status_.shutdown==0&&status_.left_alarm==0&&status_.right_alarm==0;}
public:
 explicit Lease(Profile p={}):profile_(p){profile_.verify();}
 void trip(const char* reason="safety stop"){if(!fault_)reason_=reason;fault_=true;armed_=false;v_=w_=0;deadline_=0;}
 void cancel(){armed_=false;v_=w_=0;deadline_=0;}
 void observe(const Feedback& f,double now){if(!time_ok(now))return;status_=f;observed_=now;seen_=true;if(armed_&&!healthy(now))trip("chassis interlock changed");}
 const char* arm_blocker(double now,bool stationary)const{
  if(fault_)return reason_;
  if(armed_)return "actuator already armed";
  if(!healthy(now))return "chassis feedback or interlock unavailable";
  if(!stationary)return "waiting for measured standstill";
  return "";
 }
 bool arm(uint64_t epoch,double now,bool stationary){
  if(!time_ok(now)||epoch<=epoch_||*arm_blocker(now,stationary))return false;
  epoch_=epoch;sequence_=0;armed_=true;deadline_=now+.15;v_=w_=0;return true;
 }
 bool command(uint64_t epoch,uint64_t sequence,double v,double w,double expires,double now){
  if(!time_ok(now)||fault_||!armed_||epoch!=epoch_||sequence<=sequence_)return false;
  // A delayed command must not renew an already expired owner before TX ticks.
  if(now>=deadline_){trip("motor command lease expired");return false;}
  if(!healthy(now)){trip("chassis feedback unhealthy");return false;}
  if(!std::isfinite(expires)||expires<=now||expires-now>.15){trip("invalid motor command deadline");return false;}
  try{velocity_frame(profile_,v,w);}catch(...){trip("invalid motor velocity");return false;}
  sequence_=sequence;deadline_=expires;v_=v;w_=w;return true;
 }
 Frame tick(double now){
  if(!time_ok(now))return velocity_frame(profile_,0,0);
  if(armed_&&now>=deadline_)trip("motor command lease expired");
  if(armed_&&!healthy(now))trip("chassis feedback unhealthy");
  return velocity_frame(profile_,armed_&&!fault_?v_:0,armed_&&!fault_?w_:0);
 }
 bool reset(double now,bool stationary){
  if(!time_ok(now)||armed_||!seen_||now-observed_>=.2||status_.emergency!=1||!stationary)return false;
  fault_=false;reason_="";v_=w_=0;return true;
 }
 bool armed()const{return armed_;}
 bool faulted()const{return fault_;}
 const char* fault_reason()const{return reason_;}
};
}
