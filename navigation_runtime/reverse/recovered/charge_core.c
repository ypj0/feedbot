/* Offline semantic recovery for libmove_base.so SHA256
 * 971b0895544c43f5ece5fd5cb73ac0ae03d5895046c373ffd01e4a94209e837e.
 * Offsets retain the exact extracted object layout; installed header names are documented.
 * No ROS, device I/O, or deployable vehicle control. Finite inputs only for
 * approach slice. Caller supplies effects of intercepted external routines.
 */
#include "recovered_models.h"
#include <string.h>
#include <math.h>
static double rd(const unsigned char *p, unsigned n) {
    double x; memcpy(&x,p+n,8); return x;
}
static void wd(unsigned char *p,unsigned n,double x) {memcpy(p+n,&x,8);}

/* Full foundChargePoint body, 0x103af0. getRobotPose is stubbed with no
 * memory effects; now_bits represents the packed ros::Time return value.
 * PoseStamped input is not consumed in this body. Returns GPDState SUCCESS=4 or AVOID=1. */
int recovered_found_charge(unsigned char *s, unsigned char *twist,
                           int perception_result, uint64_t now_bits) {
    if (s[0xe4] || perception_result) {
        memcpy(s+0x10c,&now_bits,8);
        return 4;
    }
    if (s[0x6f0]) return 4;
    uint32_t count; memcpy(&count,s+0x6ec,4); ++count;
    memcpy(s+0x6ec,&count,4);
    wd(twist,0,0); wd(twist,8,0);
    int32_t signed_count; memcpy(&signed_count,&count,4);
    if (signed_count <= 149 || (signed_count > 449 && signed_count <= 749)) {
        wd(twist,40,-0.3);
    } else {
        if (signed_count > 749) {count=150;memcpy(s+0x6ec,&count,4);}
        wd(twist,40,0.3);
    }
    return 1;
}
/* Full stateCallback leaf, 0x103ad0: stack argument byte +3 is
 * DriverEntry::ChargeStatus, confirmed by the installed generated header. */
void recovered_charge_state(unsigned char *s,uint8_t value) {
    s[0x6e8] = value != 0 && value != 5;
}
/* Numeric slice of approach, entry 0x1056e2 or 0x105768,
 * exit 0x1055d0. Earlier geometry and later cleanup excluded.
 * error = incoming xmm0, already normalized upstream.
 * Reads the incoming Twist; caller taskFSM preserves it across control cycles. */
void recovered_charge_approach_slice(unsigned char *s,unsigned char *t,
                                     double error,int second_entry) {
    if (second_entry) s[0x77f]=1;
    double a=fabs(error);
    if (a<=rd(s,0x708)) {
        wd(t,0,fmin(rd(t,0)+0.03,rd(s,0x730)));
        wd(t,40,0);
        return;
    }
    wd(t,0,0);
    double w=fabs(rd(t,40)),limit=rd(s,0x720);
    if (a>limit) w=fmin(w+0.06,limit);
    else {
        double target=a>0.5?0.6:0.15;
        if(w>target) w=fmax(w-(a>0.5?0.06:0.09),target);
        else w=fmin(w+0.06,target);
    }
    wd(t,40,(error<0?-1.0:1.0)*w);
}
/* startGoCharing numeric slices. rotate entry 0x103f65 (caller already
 * chose alignment), reverse entry 0x104092; both stop at 0x1040f4.
 * distance is the upstream planar distance, not a new sensor reading.
 */
void recovered_charge_final_slice(unsigned char *s,unsigned char *t,
                                  double error,double distance,int reverse) {
    if(reverse) {
        s[0x77e]=1;
        wd(t,0,fmax(rd(t,0)-0.01,rd(s,0x728)));
        wd(t,40,0);
        if(distance>rd(s,0x768)) {
            uint32_t n;memcpy(&n,s+0x778,4);++n;memcpy(s+0x778,&n,4);
        }
        wd(s,0x768,distance);
        return;
    }
    wd(t,0,0);
    double a=fabs(error),w=fabs(rd(t,40)),limit=rd(s,0x720);
    if(a>limit) w=fmin(w+0.06,limit);
    else {
        double target=a>0.5?0.6:0.05;
        double step=a>0.5?0.06:0.04;
        w=w>target?fmax(w-step,target):fmin(w+step,target);
    }
    wd(t,40,(error<0?-1.0:1.0)*w);
}
/* Full planar geometry/control semantics of approach/startGoCharing.
 * Domain: finite xy and normalized planar quaternions represented as yaw.
 * ROS strings/logging and non-planar/gimbal-lock quaternion branches excluded.
 * Literal vendor approximations 3.14, 6.28, 3.1415926 retained deliberately.
 */
static double norm(double a) {
    const double tau=6.283185307179586;
    a=fmod(fmod(a,tau)+tau,tau);
    return a>3.141592653589793?a-tau:a;
}
static void zero_twist(unsigned char *t) {wd(t,0,0);wd(t,8,0);wd(t,40,0);}
int recovered_approach_planar(unsigned char *s,unsigned char *t,const double *r,const double *d) {
    s[0x77e]=0;memset(s+0x764,0,4);wd(s,0x758,d[2]);
    double heading=d[2]+3.14;
    double diff=norm(heading-atan2(r[1]-d[1],r[0]-d[0]));
    double dx=d[0]-r[0],dy=d[1]-r[1];
    double distance=sqrt(dx*dx+dy*dy);
    double lateral=sin(diff)*distance,along=fabs(distance*cos(diff));
    if(heading<0)heading+=6.28;
    double c=cos(heading),sn=sin(heading);
    double ax=d[0]-along*c,ay=d[1]-along*sn;
    double bx=d[0]+along*c,by=d[1]+along*sn;
    double da=sqrt((ax-r[0])*(ax-r[0])+(ay-r[1])*(ay-r[1]));
    double db=sqrt((bx-r[0])*(bx-r[0])+(by-r[1])*(by-r[1]));
    if(db<=da){ax=bx;ay=by;}
    double error=norm(atan2(ay-r[1],ax-r[0])-r[2]);
    if(fabs(lateral)<rd(s,0x700)||s[0x77d]) {
        s[0x77d]=1;
        if(rd(s,0x700)<along-rd(s,0x6f8)) {
            error=norm(atan2(dy,dx)-r[2]);
            recovered_charge_approach_slice(s,t,error,0);
            return 1;
        }
        zero_twist(t);return 0;
    }
    if(rd(s,0x6f8)<=along||s[0x77f]) {
        recovered_charge_approach_slice(s,t,error,1);return 1;
    }
    wd(t,0,fmax(rd(t,0)-0.03,rd(s,0x728)));wd(t,40,0);return 1;
}
int recovered_final_planar(unsigned char *s,unsigned char *t,const double *r,const double *d) {
    double error=norm(d[2]+3.1415926-r[2]);
    double dx=d[0]-r[0],dy=d[1]-r[1],distance=sqrt(dx*dx+dy*dy);
    int rotate=0;
    if(!s[0x77e]) {
        memset(s+0x778,0,4);wd(s,0x768,distance);
        rotate=fabs(error)>rd(s,0x708);
    }
    /* With yaw_reached_ true the original extra error calculation cannot
       enter its rotation branch unless the flag changes concurrently.
       This model is single-threaded, matching the test execution. */
    recovered_charge_final_slice(s,t,error,distance,!rotate);
    if(rd(s,0x740)<=distance)return 1;
    if(fabs(rd(s,0x770)-distance)>=0.025) {
        memset(s+0x764,0,4);wd(s,0x770,distance);return 1;
    }
    uint32_t n;memcpy(&n,s+0x764,4);++n;memcpy(s+0x764,&n,4);
    int32_t v;memcpy(&v,&n,4);return v<=20;
}
