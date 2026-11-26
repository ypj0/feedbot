// Bounded lidar-only commissioning probe. No chassis access or motor commands.
#include "sdk_control.hpp"
#include "udp_input.hpp"
#include <chrono>
#include <thread>
#include <iostream>
#include <fstream>
#include <sys/file.h>
#include <fcntl.h>
static double now(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
int main(int argc,char** argv){
 if(argc!=2)return 2;
 int lock=open("/run/feedbot-lidar/transaction.lock",O_CREAT|O_RDWR,0600);
 if(lock<0||flock(lock,LOCK_EX|LOCK_NB))return 3;
 std::ifstream permit("/run/feedbot-lidar/permit");if(permit.good())return 4;
 lidar::SdkControl control("192.168.10.160","255.255.255.255",2105,2105,"LT-R2-B");
 try{
  lidar::UdpInput udp("192.168.10.160",2105);lidar::Assembler assembler;
  control.open();control.validate();udp.drain();double start=now();
  control.measurement(true);std::cout<<"measurement_ack_s="<<now()-start<<std::endl;
  control.streaming(true);std::cout<<"streaming_ack_s="<<now()-start<<std::endl;
  unsigned blocks=0,scans=0;
  while(now()-start<10){
   lidar::Block b;double stamp;
   if(udp.read(b,stamp)){
    if(!blocks++)std::cout<<"first_packet_s="<<now()-start<<" frame="<<b.frame<<" index="<<int(b.index)<<" count="<<int(b.count)<<std::endl;
    lidar::Scan scan;
    if(assembler.add(b,stamp,scan)){
     if(!scans++)std::cout<<"first_scan_s="<<now()-start<<" beams="<<scan.ranges.size()<<std::endl;
     if(scans>=10)break;
    }
   }else std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  std::cout<<"blocks="<<blocks<<" scans="<<scans<<" discarded="<<assembler.discarded<<std::endl;
  control.streaming(false);control.measurement(false);control.close();return scans?0:1;
 }catch(const std::exception& e){std::cerr<<e.what()<<std::endl;try{control.streaming(false);control.measurement(false);}catch(...){}control.close();return 1;}
}
