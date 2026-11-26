#include "stationarity.hpp"
#include "lease.hpp"
#include <cassert>
#include <iostream>
using namespace feedbot_chassis;
int main(){
 Stationarity s;
 assert(!s.ready(1));
 for(int i=0;i<12;i++)s.observe(1+i*.01,.00045,-.00196,0,true);
 assert(s.ready(1.11));assert(!s.ready(1.22));
 Feedback f{0,1,83,0,0,0,0,2,0,441,0,0,0,-19};
 Lease l;l.observe(f,1.11);assert(l.arm(1,1.11,s.ready(1.11)));
 l.cancel();s.observe(1.12,.1,0,0,true);assert(!s.ready(1.12));assert(!l.arm(2,1.12,s.ready(1.12)));
 for(int i=0;i<12;i++)s.observe(2+i*.01,0,0,0,true);
 assert(s.ready(2.11));s.observe(2.12,0,0,.1,true);assert(!s.ready(2.12));
 for(int i=0;i<12;i++)s.observe(3+i*.01,0,0,0,true);
 s.observe(3.12,0,.1,0,true);assert(!s.ready(3.12));
 s.observe(4,0,0,0,true);s.observe(4,0,0,0,true);assert(!s.ready(4.11));
 s.observe(5,NAN,0,0,true);assert(!s.ready(5));
 s.observe(6,0,0,0,false);assert(!s.ready(6));
 f.emergency=1;l.observe(f,7);assert(!l.arm(3,7,true));
 std::cout<<"PASS measured standstill, raw-field independence, motion and stale rejection\n";
}
