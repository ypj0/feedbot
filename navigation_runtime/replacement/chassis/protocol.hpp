#pragma once
#include <array>
#include <cstdint>
#include <cmath>
#include <stdexcept>
namespace feedbot_chassis {
using Frame=std::array<uint8_t,40>;
inline void be16(Frame& f,unsigned i,int value){f[i]=uint16_t(value)>>8;f[i+1]=uint16_t(value);}
inline int16_t signed_be(const Frame& f,unsigned i){unsigned u=(unsigned(f[i])<<8)|f[i+1];return u<32768?int16_t(u):int16_t(int(u)-65536);}
inline void checksum(Frame& f){unsigned sum=0;for(unsigned i=0;i<37;i++)sum+=f[i];f[37]=uint8_t(0-sum);}
inline bool valid(const Frame& f){
 if(f[0]!=127||f[1]!=127||f[2]!=40||f[3]<1||f[3]>3||f[38]!=13||f[39]!=10)return false;
 unsigned sum=0;for(unsigned i=0;i<38;i++)sum+=f[i];return (sum&255)==0;
}
inline constexpr Frame kReviewedStationary{{127,127,40,2,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,1,7,20,20,20,170,21,224,1,199,10,0,0,0,0,0,33,13,10}};
struct Profile {
 // One reviewed hardware profile. Every nonvelocity byte is frozen, including
 // auxiliary configuration whose physical semantics are not fully recovered.
 Frame stationary=kReviewedStationary;
 void verify()const {
  if(stationary!=kReviewedStationary)throw std::invalid_argument("unreviewed chassis profile");
 }
};
inline Frame velocity_frame(const Profile& p,double v,double w){
 p.verify();
 // Reviewed protocol envelope. Application commissioning limits are a separate owner policy.
 if(!std::isfinite(v)||!std::isfinite(w)||std::abs(v)>.5||std::abs(w)>1.)throw std::invalid_argument("velocity outside reviewed envelope");
 Frame f=p.stationary;be16(f,10,int(std::lround(v*1000)));be16(f,12,int(std::lround(w*1000)));checksum(f);return f;
}
struct Feedback {
 int emergency,initialized,power,charge,left_alarm,right_alarm,motor_free,mode,shutdown,version;
 int left_encoder,right_encoder,v_raw,w_raw;
};
inline Feedback feedback(const Frame& f){
 if(!valid(f)||f[3]!=2||f[4]!=0x60)throw std::invalid_argument("invalid runtime feedback");
 return {f[5],f[6],f[7],f[8],f[9],f[10],f[27],f[33],f[28],signed_be(f,35),signed_be(f,11),signed_be(f,13),signed_be(f,29),signed_be(f,31)};
}
}
