#include "packet.hpp"
#include <fstream>
#include <iostream>
#include <iomanip>
static void array(const std::vector<float>& v){std::cout<<'[';for(size_t i=0;i<v.size();i++){if(i)std::cout<<',';if(std::isfinite(v[i]))std::cout<<v[i];else std::cout<<"null";}std::cout<<']';}
int main(int argc,char** argv){try{if(argc!=2)return 2;std::ifstream f(argv[1],std::ios::binary);if(!f)throw std::runtime_error("open");lidar::Assembler a;lidar::Scan scan;std::cout<<std::setprecision(17);
while(f.peek()!=EOF){double t;uint32_t n;f.read(reinterpret_cast<char*>(&t),8);f.read(reinterpret_cast<char*>(&n),4);if(!f||n>65535)throw std::runtime_error("record header");std::vector<uint8_t> p(n);f.read(reinterpret_cast<char*>(p.data()),n);if(!f)throw std::runtime_error("record body");const uint8_t* data;size_t size;if(!lidar::udp(p,data,size))continue;auto b=lidar::decode(data,size);if(!a.add(b,t,scan))continue;
std::cout<<"{\"frame\":"<<b.frame<<",\"received\":"<<t<<",\"stamp\":"<<scan.stamp<<",\"scan_time\":"<<scan.period<<",\"time_increment\":"<<scan.increment<<",\"angle_min\":"<<scan.angle_min<<",\"angle_max\":"<<scan.angle_max<<",\"ranges\":";array(scan.ranges);std::cout<<",\"intensities\":";array(scan.intensities);std::cout<<"}\n";}
}catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 2;}}
