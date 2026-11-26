#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
namespace feedbot_imu {
struct Snapshot {
 std::array<double,3> acceleration{},angular_velocity{},rpy{};
 std::array<double,4> quaternion{{0,0,0,1}}; // ROS x,y,z,w
 std::array<double,3> received{}; // Separate acceleration/gyro/orientation times.
 bool ready=false;
};
// WIT 0x55/0x51..53 packets, matching the inspected factory SingleIMU driver.
// Read-only decoder: does not open a device, configure IMU, fuse or publish TF.
class Decoder {
 std::array<uint8_t,11> bytes_{};unsigned count_=0;
 Snapshot state_{};std::array<bool,3> seen_{};
 double last_=0;
 static int signed_le(uint8_t a,uint8_t b){int n=int(a)|(int(b)<<8);return n<32768?n:n-65536;}
public:
 uint64_t accepted=0,rejected=0;
 void push(uint8_t byte,double receipt){
  if(!std::isfinite(receipt)||receipt<last_){state_.ready=false;seen_={};count_=0;throw std::invalid_argument("IMU receipt clock discontinuity");}
  last_=receipt;bytes_[count_++]=byte;
  while(count_&&(bytes_[0]!=0x55||(count_>1&&(bytes_[1]<0x51||bytes_[1]>0x53)))){
   std::move(bytes_.begin()+1,bytes_.begin()+count_,bytes_.begin());--count_;++rejected;
  }
  if(count_!=11)return;
  unsigned sum=0;for(unsigned i=0;i<10;i++)sum+=bytes_[i];
  if((sum&255)!=bytes_[10]){
   std::move(bytes_.begin()+1,bytes_.end(),bytes_.begin());--count_;++rejected;return;
  }
  unsigned kind=bytes_[1]-0x51;std::array<double,3> values{};const double pi=std::acos(-1.);
  for(unsigned i=0;i<3;i++){
   double raw=double(signed_le(bytes_[2+i*2],bytes_[3+i*2]))/32768.;
   values[i]=kind==0?raw*16.*9.80665:kind==1?raw*2000.*pi/180.:raw*pi;
  }
  if(kind==0)state_.acceleration=values;
  else if(kind==1)state_.angular_velocity=values;
  else {
   state_.rpy=values;
   double sr=std::sin(values[0]/2),cr=std::cos(values[0]/2);
   double sp=std::sin(values[1]/2),cp=std::cos(values[1]/2);
   double sy=std::sin(values[2]/2),cy=std::cos(values[2]/2);
   state_.quaternion={sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy};
  }
  state_.received[kind]=receipt;seen_[kind]=true;count_=0;++accepted;
 }
 Snapshot snapshot(double time,double max_age,double max_skew)const{
  Snapshot out=state_;
  out.ready=std::isfinite(time)&&std::isfinite(max_age)&&std::isfinite(max_skew)&&max_age>0&&max_skew>=0&&
            time>=last_&&seen_[0]&&seen_[1]&&seen_[2];
  auto bounds=std::minmax_element(out.received.begin(),out.received.end());
  out.ready=out.ready&&time-*bounds.first<max_age&&*bounds.second-*bounds.first<=max_skew;
  return out;
 }
};
}
