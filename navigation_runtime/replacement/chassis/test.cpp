#include "lease.hpp"
#include <cassert>
#include <iostream>
using namespace feedbot_chassis;
int main(){
 Profile p;p.verify();assert(velocity_frame(p,0,0)==p.stationary);
 auto f=velocity_frame(p,.5,-1);assert(valid(f)&&signed_be(f,10)==500&&signed_be(f,12)==-1000);
 for(unsigned i=0;i<40;i++)if(i<10||(i>13&&i!=37))assert(f[i]==p.stationary[i]);
 bool threw=false;try{velocity_frame(p,NAN,0);}catch(...){threw=true;}assert(threw);
 Feedback healthy{0,1,83,0,0,0,0,2,0,441,0,0,0,0};
 Lease s;assert(s.tick(1)==p.stationary);assert(!s.arm(1,1,true));s.observe(healthy,1);assert(s.arm(1,1,true));assert(s.command(1,1,.1,.2,1.1,1));assert(signed_be(s.tick(1.05),10)==100);
 // Receive stops entirely: TX tick still independently emits exact stationary frame at expiry.
 assert(s.tick(1.11)==p.stationary);assert(s.faulted());assert(std::string(s.fault_reason())=="motor command lease expired");s.trip("later generic stop");assert(std::string(s.fault_reason())=="motor command lease expired");s.observe(healthy,1.12);assert(!s.arm(2,1.12,true));assert(!s.reset(1.12,true));
 healthy.emergency=1;s.observe(healthy,1.13);assert(s.reset(1.13,true));healthy.emergency=0;s.observe(healthy,1.14);assert(s.arm(2,1.14,true));assert(s.command(2,1,.1,0,1.24,1.14));assert(!s.command(1,2,.1,0,1.25,1.15));assert(signed_be(s.tick(1.16),10)==100);assert(s.tick(1.25)==p.stationary);
 Lease t;t.observe(healthy,2);assert(t.arm(1,2,true));assert(t.command(1,1,.1,0,2.1,2));healthy.emergency=1;t.observe(healthy,2.01);assert(t.tick(2.02)==p.stationary);assert(t.faulted());
 healthy.emergency=0;Lease cancelled;cancelled.observe(healthy,3);assert(cancelled.arm(1,3,true));
 assert(cancelled.command(1,1,.1,0,3.1,3));cancelled.cancel();
 assert(!cancelled.faulted()&&cancelled.tick(3.01)==p.stationary);
 assert(cancelled.arm(2,3.02,true));assert(!cancelled.command(1,2,.1,0,3.1,3.02));
 std::cout<<"PASS offline full-frame profile, expiry without RX, stale epoch, physical reset, emergency, explicit cancel/rearm\n";
}
