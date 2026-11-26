#include "transport.hpp"
#include <cassert>
#include <chrono>
#include <iostream>
#include <thread>
#include <termios.h>
#ifdef __APPLE__
#include <util.h>
#else
#include <pty.h>
#endif
using namespace feedbot_chassis;
int main(){
 int master,slave;assert(openpty(&master,&slave,nullptr,nullptr,nullptr)==0);
 termios settings{};assert(tcgetattr(slave,&settings)==0);cfmakeraw(&settings);
 // O_NONBLOCK plus VMIN=1: zero-length read represents disconnect, not idle UART.
 settings.c_cc[VMIN]=1;settings.c_cc[VTIME]=0;assert(tcsetattr(slave,TCSANOW,&settings)==0);
 for(int fd:{master,slave})assert(fcntl(fd,F_SETFL,O_NONBLOCK)==0);
 Transport t(slave);
 Frame rx=kReviewedStationary;rx[4]=0x60;rx[5]=1;rx[6]=1;rx[33]=2;be16(rx,35,441);checksum(rx);
 // Deliver one byte per reactor cycle, exercising real tty buffering/fragmentation.
 for(size_t i=0;i<rx.size();i++){
  assert(write(master,&rx[i],1)==1);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  t.step(1+i*.002);
 }
 for(int i=0;i<10&&t.feedback_count()==0;i++){
  std::this_thread::sleep_for(std::chrono::milliseconds(1));t.step(1.08+i*.002);
 }
 assert(t.feedback_count()==1&&!t.failed());
 assert(!t.arm(1,1.11,true)); // Physical emergency prevents arming.
 std::array<uint8_t,1024> out{};ssize_t count=read(master,out.data(),out.size());
 assert(count>=40&&count%40==0);
 for(ssize_t pos=0;pos<count;pos+=40){Frame f{};std::copy_n(out.data()+pos,40,f.begin());assert(f==kReviewedStationary);}
 close(master);t.step(1.12);assert(t.failed());close(slave);
 std::cout<<"PASS real PTY fragmentation, periodic stationary TX, emergency refusal, disconnect\n";
}
