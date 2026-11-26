#include "transport.hpp"
#include <cassert>
#include <csignal>
#include <iostream>
#include <sys/socket.h>
using namespace feedbot_chassis;
ssize_t short_write(int fd,const void* p,size_t n){return write(fd,p,std::min(n,size_t(7)));}
struct Pair {
 int fd[2];
 Pair(){assert(socketpair(AF_UNIX,SOCK_STREAM,0,fd)==0);for(int s:fd)assert(fcntl(s,F_SETFL,O_NONBLOCK)==0);}
 ~Pair(){for(int s:fd)close(s);}
 void feedback_to_driver(bool emergency=false){
  Frame f{};f[0]=f[1]=127;f[2]=40;f[3]=2;f[4]=0x60;f[5]=emergency;f[6]=1;f[7]=82;f[33]=2;
  be16(f,35,441);f[38]=13;f[39]=10;checksum(f);
  assert(write(fd[1],f.data(),f.size())==40);
 }
 Frame output(){Frame f{};assert(read(fd[1],f.data(),40)==40);assert(valid(f));return f;}
};
int main(){
 signal(SIGPIPE,SIG_IGN);
 Pair p;Transport t(p.fd[0]);p.feedback_to_driver();t.step(1);assert(p.output()==kReviewedStationary);
 assert(t.arm(1,1,true));assert(t.command(1,1,.1,.2,1.1,1));t.step(1.03);assert(signed_be(p.output(),10)==100);
 // Entire RX stream disappears: TX keeps running and reaches zero at expiry.
 t.step(1.12);assert(p.output()==kReviewedStationary&&t.faulted()&&!t.failed());
 p.feedback_to_driver();t.step(1.15);assert(p.output()==kReviewedStationary);assert(!t.arm(2,1.15,true));
 p.feedback_to_driver(true);t.step(1.18);p.output();assert(t.reset(1.18,true));
 p.feedback_to_driver();t.step(1.21);p.output();assert(t.arm(2,1.21,true));
 // Late incoming command cannot resurrect an expired owner between ticks.
 assert(!t.command(2,1,.1,0,1.5,1.37));assert(t.faulted());
 // Arbitrary garbage/overlapping headers and corrupted frames do not poison stream.
 FeedbackStream parser;unsigned deliveries=0;Frame f=kReviewedStationary;f[4]=0x60;checksum(f);
 for(int i=0;i<10000;i++)parser.push(0,[&](Feedback){++deliveries;});
 parser.push(127,[&](Feedback){++deliveries;});
 Frame broken=f;broken[20]^=1;
 for(auto b:broken)parser.push(b,[&](Feedback){++deliveries;});
 for(auto b:f)parser.push(b,[&](Feedback){++deliveries;});
 assert(deliveries==1&&parser.accepted==1);
 // A permanently blocked output trips rather than building an unbounded queue.
 Pair blocked;Transport b(blocked.fd[0]);char junk[4096]{};
 while(write(blocked.fd[0],junk,sizeof(junk))>0){}
 assert(errno==EAGAIN||errno==EWOULDBLOCK);
 blocked.feedback_to_driver();b.step(2);assert(!b.failed());b.step(2.05);assert(b.failed());
 assert(!b.arm(1,2.05,true));
 // Restart has no inherited command or arming state.
 Pair fresh;Transport restart(fresh.fd[0]);fresh.feedback_to_driver();restart.step(3);
 assert(fresh.output()==kReviewedStationary);assert(!restart.command(2,10,.1,0,3.1,3));
 // Short writes preserve exact frame boundaries when the target remains current.
 Pair partial;Transport split(partial.fd[0],short_write);partial.feedback_to_driver();
 for(int i=0;i<6;i++)split.step(4+i*.001);
 assert(partial.output()==kReviewedStationary&&!split.failed());
 assert(split.arm(1,4.006,true));assert(split.command(1,1,.1,0,4.03,4.006));
 split.step(4.021); // Seven bytes of a nonzero command have been emitted.
 split.step(4.031); // Expired: never append a zero frame to that unfinished frame.
 assert(split.failed());char remaining[80];assert(read(partial.fd[1],remaining,sizeof(remaining))==7);
 std::cout<<"PASS bounded stream recovery, RX loss/independent TX, expiry before renewal, blocked output, restart disarmed\n";
}
