#include "wit_decoder.hpp"
#include <cassert>
#include <iostream>
using namespace feedbot_imu;
std::array<uint8_t,11> frame(unsigned kind,int x,int y,int z){
 std::array<uint8_t,11> b{};b[0]=0x55;b[1]=kind;
 int values[]={x,y,z};for(unsigned i=0;i<3;i++){b[2+2*i]=uint16_t(values[i]);b[3+2*i]=uint16_t(values[i])>>8;}
 for(unsigned i=0;i<10;i++)b[10]+=b[i];
 return b;
}
void feed(Decoder& d,std::array<uint8_t,11> f,double t){for(auto byte:f)d.push(byte,t);}
void near(double a,double b){assert(std::abs(a-b)<1e-12);}
int main(){
 Decoder d;assert(!d.snapshot(1,.1,.02).ready);
 feed(d,frame(0x51,0,0,2048),1);feed(d,frame(0x52,0,0,-16384),1.001);
 assert(!d.snapshot(1.001,.1,.02).ready);
 feed(d,frame(0x53,0,0,16384),1.002);auto s=d.snapshot(1.003,.1,.02);assert(s.ready);
 near(s.acceleration[2],9.80665);near(s.angular_velocity[2],-1000*std::acos(-1.)/180.);
 near(s.quaternion[2],std::sqrt(.5));near(s.quaternion[3],std::sqrt(.5));
 assert(!d.snapshot(1.2,.1,.02).ready);
 // A fresh angle does not refresh old acceleration or gyro samples.
 feed(d,frame(0x53,0,0,0),1.21);assert(!d.snapshot(1.21,.1,.02).ready);
 auto bad=frame(0x52,1,2,3);bad[10]^=1;feed(d,bad,1.22);
 for(int i=0;i<10000;i++)d.push(0,1.23);
 feed(d,frame(0x51,-32768,32767,0),1.24);feed(d,frame(0x52,0,0,0),1.241);
 feed(d,frame(0x53,0,0,0),1.242);s=d.snapshot(1.243,.1,.02);assert(s.ready);
 near(s.acceleration[0],-16*9.80665);near(s.quaternion[3],1.);
 bool threw=false;try{d.push(0,1);}catch(const std::invalid_argument&){threw=true;}assert(threw);
 assert(!d.snapshot(1.244,.1,.02).ready);
 std::cout<<"PASS IMU frame recovery, SI units, quaternion, per-channel age/skew, clock reversal\n";
}
