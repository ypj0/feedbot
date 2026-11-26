#pragma once
#include <cmath>
namespace feedbot_chassis {
// Physical units, independent of undocumented firmware raw velocity fields.
// Conservative commissioning standstill: <=5 mm/s, <=0.02 rad/s for 100 ms.
class Stationarity {
 double since_=-1,last_=-1;
public:
 void observe(double stamp,double v,double wheel_w,double gyro_w,bool valid){
  if(!valid||!std::isfinite(stamp)||!std::isfinite(v)||!std::isfinite(wheel_w)||!std::isfinite(gyro_w)||
     std::abs(v)>.005||std::abs(wheel_w)>.02||std::abs(gyro_w)>.02||stamp<last_){since_=-1;last_=-1;return;}
  if(last_<0||stamp-last_>=.1)since_=stamp;
  last_=stamp;
 }
 bool ready(double now)const{return since_>=0&&last_-since_>=.1&&now>=last_&&now-last_<.1;}
};
}
