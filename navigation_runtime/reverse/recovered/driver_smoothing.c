/* Recovered from robot_driver 0x3f680. Offline semantic model, not a driver.
 * Explicit state replaces original process-global static floats/time.
 * Finite inputs, positive range, nonnegative dt and rates; no fast-math/FMA.
 * ROS time acquisition/first-call initialization is outside this model. */
#include "recovered_models.h"
#include <math.h>
static float axis(float previous, int16_t target, float filter,
                  float range, float dt, double positive, double negative) {
    float delta=(float)target-previous;
    float factor=(fabsf(delta)/range)*(1.0f-filter)+filter;
    if(factor<=1.0f) delta*=factor;
    delta=(delta+previous)-previous;
    if(delta<=0.0f) {
        float lower=-(float)negative*dt;
        if(delta<lower) delta=lower;
    } else if((double)delta>(double)dt*positive) delta=(float)((double)dt*positive);
    return previous+delta;
}
uint32_t recovered_smooth_pwm(float state[2], int16_t v, int16_t w,
                             int32_t sec, int32_t nsec, float cap, float range,
                             const double params[4]) {
    float dt=(float)((double)sec+(double)nsec*1e-9);
    if(dt>cap) dt=cap;
    int stopped=(v==0 && w==0);
    float filter=(float)params[stopped?3:0];
    double negative=params[stopped?2:1];
    state[0]=axis(state[0],v,filter,range,dt,params[1],negative);
    state[1]=axis(state[1],w,filter,range,dt,params[1],negative);
    return ((uint32_t)(int32_t)state[0]&0xffffu)|((uint32_t)(int32_t)state[1]<<16);
}
