#include "sdk_control.hpp"
#include <iostream>
int main(int argc,char** argv){try{if(argc!=2)return 2;lidar::SdkControl d("127.0.0.1","127.0.0.1",uint16_t(std::stoi(argv[1])));lidar::Lifecycle l(d);l.claim();if(!l.configure()||!l.activate(1)||l.ready()||!l.frame(l.epoch(),1.1,1.1)||!l.stop(true)||!l.activate(2)||!l.stop(false)||!l.release())throw std::runtime_error("fixture lifecycle failed");std::cout<<"PASS SDK TCP configure/start/sleep/wake/stop/release; loopback only\n";}catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 2;}}
