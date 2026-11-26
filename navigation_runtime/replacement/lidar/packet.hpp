#pragma once
#include <cstdint>
#include <vector>
#include <stdexcept>
#include <cmath>
#include <limits>
#include <algorithm>
namespace lidar {
inline uint16_t le(const uint8_t* p){return p[0]|uint16_t(p[1])<<8;}
inline uint16_t be(const uint8_t* p){return uint16_t(p[0])<<8|p[1];}
inline uint32_t le32(const uint8_t* p){return le(p)|uint32_t(le(p+2))<<16;}
struct Block {uint16_t frame;uint8_t index,count;uint32_t stamp;std::vector<uint16_t> ranges,intensities;};
inline Block decode(const uint8_t* p,size_t n){
 if(n<16||le(p)!=65535)throw std::runtime_error("OOB header");
 uint16_t crc=65535;
 for(size_t i=0;i<n;i++){crc^=uint16_t(i==12||i==13?0:p[i])<<8;for(int bit=0;bit<8;bit++)crc=crc&0x8000?uint16_t((crc<<1)^0x1021):uint16_t(crc<<1);}
 if(crc!=le(p+12))throw std::runtime_error("OOB CRC");
 uint16_t length=le(p+6),flags=le(p+14);unsigned width=flags&1?2:1;
 // This candidate is explicitly for the observed LT-R2 360-degree format.
 if((flags&~0x101)||!(flags&0x100)||!p[5]||p[5]>32||p[4]>=p[5]||!length||length>4096||n!=16+size_t(length)*(2+width))throw std::runtime_error("OOB layout");
 Block b{le(p+2),p[4],p[5],le32(p+8),{}, {}};b.ranges.resize(length);b.intensities.resize(length);
 for(size_t i=0;i<length;i++){b.ranges[i]=le(p+16+2*i);auto q=p+16+2*length+width*i;b.intensities[i]=width==2?le(q):*q;}return b;
}
inline bool udp(const std::vector<uint8_t>& p,const uint8_t*& data,size_t& n){
 if(p.size()<42||be(p.data()+12)!=0x800||p[14]>>4!=4||p[23]!=17)return false;
 if(p[26]!=192||p[27]!=168||p[28]!=10||p[29]!=160)return false;
 size_t ihl=(p[14]&15)*4,total=be(p.data()+16);if(ihl<20||total<ihl+8||p.size()<14+total||(be(p.data()+20)&0x3fff))throw std::runtime_error("IP length/fragment");
 size_t o=14+ihl;size_t len=be(p.data()+o+4);if(len<8||len!=total-ihl||be(p.data()+o)!=2105||be(p.data()+o+2)!=2105)throw std::runtime_error("UDP layout");data=p.data()+o+8;n=len-8;return true;
}
struct Scan {double stamp,period,increment,angle_min,angle_max;std::vector<float> ranges,intensities;};
class Assembler {
 bool active=false;uint16_t frame=0;uint8_t next=0,count=0;size_t length=0;uint32_t first_device=0;double first_host=0;
 std::vector<uint16_t> ranges,intensities;
 public:
 uint64_t discarded=0;
 bool add(const Block& b,double host,Scan& out){
  if(b.index==0){if(active)discarded++;active=true;frame=b.frame;next=0;count=b.count;length=b.ranges.size();first_device=b.stamp;first_host=host;ranges.clear();intensities.clear();}
  if(!active)return false;
  if(b.frame!=frame||b.index!=next||b.count!=count||b.ranges.size()!=length||b.intensities.size()!=length){active=false;discarded++;return false;}
  ranges.insert(ranges.end(),b.ranges.begin(),b.ranges.end());intensities.insert(intensities.end(),b.intensities.begin(),b.intensities.end());next++;
  if(next<count)return false;
  active=false;
  if(count<2||count%2)throw std::runtime_error("block geometry");
  double elapsed=uint32_t(b.stamp-first_device)*1e-6;
  double period=elapsed*count/(count-1);if(period<.025||period>.15||host<first_host||host-first_host>.2)throw std::runtime_error("scan timing");
  const double pi=3.14159265358979323846,step=2*pi/ranges.size();int lo=int(std::ceil(-1.65/step)),hi=int(std::floor(1.65/step)),start=int(ranges.size()/2)+lo;
  out.period=period;out.increment=period/ranges.size();out.angle_min=lo*step;out.angle_max=hi*step;
  // Host-reception estimate only; hardware clock synchronization is not yet calibrated.
  out.stamp=first_host-period/count+start*out.increment;
  out.ranges.clear();out.intensities.clear();for(int i=lo;i<=hi;i++){size_t j=ranges.size()/2+i;out.ranges.push_back(ranges[j]?ranges[j]*.002f:std::numeric_limits<float>::infinity());out.intensities.push_back(ranges[j]?intensities[j]:0);}
  return true;
 }
};
}
