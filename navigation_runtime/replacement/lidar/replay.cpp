#include "packet.hpp"
#include <fstream>
#include <iostream>
int main(int argc,char** argv){try{if(argc!=2)return 2;std::ifstream f(argv[1],std::ios::binary);if(!f)throw std::runtime_error("open");lidar::Assembler a;uint64_t packets=0,scans=0;double first=0,last=0;lidar::Scan scan;
while(f.peek()!=EOF){double t;uint32_t n;f.read(reinterpret_cast<char*>(&t),8);f.read(reinterpret_cast<char*>(&n),4);if(!f||n>65535)throw std::runtime_error("record header");std::vector<uint8_t> p(n);f.read(reinterpret_cast<char*>(p.data()),n);if(!f)throw std::runtime_error("record body");const uint8_t* data;size_t size;if(!lidar::udp(p,data,size))continue;auto b=lidar::decode(data,size);packets++;if(a.add(b,t,scan)){if(!first)first=t;last=t;scans++;}}
std::cout<<"{\"packets\":"<<packets<<",\"scans\":"<<scans<<",\"discarded_partial\":"<<a.discarded<<",\"hz\":"<<(scans-1)/(last-first)<<",\"beams\":"<<scan.ranges.size()<<",\"period\":"<<scan.period<<",\"angle_min\":"<<scan.angle_min<<",\"angle_max\":"<<scan.angle_max<<"}\n";
}catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 2;}}
