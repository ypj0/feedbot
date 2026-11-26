/* Vendor DWA linePID_control 0x317e0 / dirPID_control 0x31bc0.
 * Finite numeric inputs; ROS logging has no modeled state mutation. */
#include "recovered_models.h"
#include <string.h>
#include <math.h>
static float rf(const unsigned char *p){float x;memcpy(&x,p,4);return x;}
static void wf(unsigned char *p,float x){memcpy(p,&x,4);}
float recovered_dwa_pid(unsigned char *object,float command,int direction) {
    unsigned char *p=object+(direction?0x900:0x8d4);
    double speed;memcpy(&speed,object+0x970,8);
    float threshold=speed>=.7?.7f:(speed>.4?.35f:.3f);
    float actual=rf(p+4),err=command-actual;
    wf(p,command);wf(p+8,err);
    if(command==0){memset(p+4,0,16);wf(p+32,0);actual=0;err=0;}
    float integral=rf(p+32),enabled=0;
    if(fabsf(err)<=threshold){
        enabled=1;
        int accumulate=actual>rf(p+36)?err<0:(actual<rf(p+40)?err>0:1);
        if(accumulate){integral+=err;wf(p+32,integral);}
    }
    float derivative=err-rf(p+16);
    float base=rf(p+20)*err+actual;
    float integral_part=(enabled*rf(p+24))*integral;
    float result=derivative*rf(p+28)+(integral_part+base);
    wf(p+4,result);wf(p+16,err);return result;
}
