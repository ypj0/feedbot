#include "packet.hpp"
#include <fstream>
#include <iostream>
#include <functional>
static void require(bool b){if(!b)throw std::runtime_error("test failed");}
static void reject(std::function<void()> f){bool bad=false;try{f();}catch(const std::exception&){bad=true;}require(bad);}
int main(int argc,char** argv){try{require(argc==2);std::ifstream f(argv[1],std::ios::binary);std::vector<lidar::Block> blocks;std::vector<double> times;std::vector<uint8_t> packet;
while(f.peek()!=EOF&&blocks.size()<16){double t;uint32_t n;f.read(reinterpret_cast<char*>(&t),8);f.read(reinterpret_cast<char*>(&n),4);require(bool(f)&&n<=65535);std::vector<uint8_t> p(n);f.read(reinterpret_cast<char*>(p.data()),n);require(bool(f));const uint8_t* d;size_t z;if(!lidar::udp(p,d,z))continue;auto b=lidar::decode(d,z);if(blocks.empty()&&b.index)continue;blocks.push_back(b);times.push_back(t);packet.assign(d,d+z);}
require(blocks.size()==16);
for(size_t n=0;n<packet.size();n++)reject([&]{lidar::decode(packet.data(),n);});
for(size_t i=0;i<packet.size();i++){auto p=packet;p[i]^=1;reject([&]{lidar::decode(p.data(),p.size());});}
lidar::Scan scan;lidar::Assembler ok;int count=0;for(size_t i=0;i<16;i++)count+=ok.add(blocks[i],times[i],scan);require(count==2&&scan.ranges.size()==1075);require(std::abs(scan.angle_max-scan.angle_min-(scan.ranges.size()-1)*2*3.14159265358979323846/2048)<1e-10);
lidar::Assembler missing;count=0;for(size_t i=0;i<16;i++)if(i!=3)count+=missing.add(blocks[i],times[i],scan);require(count==1&&missing.discarded==1);
lidar::Assembler duplicate;count=0;for(size_t i=0;i<16;i++){count+=duplicate.add(blocks[i],times[i],scan);if(i==2)count+=duplicate.add(blocks[i],times[i],scan);}require(count==1&&duplicate.discarded==1);
std::cout<<"PASS real UDP CRC, all truncation boundaries, every single-byte corruption, incomplete/duplicate block rejection, next-frame recovery, beam geometry\n";
}catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 2;}}
