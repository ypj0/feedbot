#include "odometry.hpp"
#include <cassert>
#include <iostream>
using namespace feedbot_chassis;
Feedback sample(int left,int right){return {0,1,62,0,0,0,0,2,0,441,left,right,100,-200};}
void close_to(double a,double b){assert(std::abs(a-b)<1e-10);}
int main(){
 auto s=wheel_sample(sample(2000,-2000),{});close_to(s.left_m,.02);close_to(s.right_m,-.02);
 close_to(s.v_mps,.1);close_to(s.w_radps,.2);
 s=wheel_sample(sample(2001,-2001),{2,2,.455});close_to(s.left_m,.01);close_to(s.right_m,-.01);
 // Constant repeated increments are real new travel, not duplicated counters.
 WheelOdometry straight;straight.begin(1,0,1);
 for(unsigned i=1;i<=50;i++)assert(straight.update(1,i,1+i*.02,sample(1000,1000)));
 close_to(straight.pose().x,.5);close_to(straight.pose().y,0);
 WheelOdometry spin;spin.begin(1,0,1);assert(spin.update(1,1,1.02,sample(-1000,1000)));
 close_to(spin.pose().x,0);close_to(spin.pose().y,0);close_to(spin.pose().yaw,.02/.455);
 // Analytic circular arc and subdivision invariance.
 WheelOdometry coarse({1,1,.5}),fine({1,1,.5});coarse.begin(1,0,1);fine.begin(1,0,1);
 assert(coarse.update(1,1,1.1,sample(10000,20000)));
 for(unsigned i=1;i<=10;i++)assert(fine.update(1,i,1+i*.01,sample(1000,2000)));
 close_to(coarse.pose().x,.75*std::sin(.2));close_to(coarse.pose().y,.75*(1-std::cos(.2)));
 close_to(coarse.pose().x,fine.pose().x);close_to(coarse.pose().y,fine.pose().y);
 // Twist must follow encoder increments, not biased vendor speed fields.
 WheelOdometry measured;measured.begin(1,0,1.);
 assert(!measured.velocity_valid());
 for(unsigned i=1;i<=20;i++)assert(measured.update(1,i,1.+i*.02,sample(200,200)));
 assert(measured.velocity_valid());close_to(measured.linear_velocity(),.1);close_to(measured.angular_velocity(),0.);
 close_to(measured.pose().x,.04);
 // The firmware's speed field stays 0.1 even when increments become zero.
 for(unsigned i=21;i<=30;i++)assert(measured.update(1,i,1.+i*.02,sample(0,0)));
 close_to(measured.linear_velocity(),0.);close_to(measured.angular_velocity(),0.);
 WheelOdometry reverse;reverse.begin(1,0,1.);
 for(unsigned i=1;i<=20;i++)assert(reverse.update(1,i,1.+i*.02,sample(-200,-200)));
 close_to(reverse.linear_velocity(),-.1);
 WheelOdometry turn;turn.begin(1,0,1.);
 for(unsigned i=1;i<=20;i++)assert(turn.update(1,i,1.+i*.02,sample(-200,200)));
 close_to(turn.linear_velocity(),0.);close_to(turn.angular_velocity(),.2/.455);
 // Jittered receipt periods preserve distance/time, including closely batched frames.
 WheelOdometry jitter;jitter.begin(1,0,1.);double tm=1.;
 for(unsigned i=1;i<=20;i++){double dt=i%2?.03:.01;tm+=dt;int increment=i%2?300:100;assert(jitter.update(1,i,tm,sample(increment,increment)));}
 close_to(jitter.linear_velocity(),.1);
 // Lost stream continuity invalidates the estimate; no extrapolation or auto resume.
 assert(!straight.update(1,52,2.04,sample(1000,1000)));assert(!straight.valid());assert(!straight.velocity_valid());
 assert(!straight.update(1,51,2.06,sample(1000,1000)));close_to(straight.pose().x,.5);
 straight.begin(2,0,3);assert(straight.valid());close_to(straight.pose().x,0);
 assert(!straight.update(2,1,3.3,sample(1000,1000)));
 std::cout<<"PASS feedback units/sign, incremental semantics, straight/spin/arc, subdivision, stream discontinuity\n";
}
