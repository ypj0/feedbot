/* Original Thumb target routine 0x0800d170, finite sensor input.
 * RAM 0x200000e4 physical meaning unproven. Do not label as obstacle distance.
 * mode is the intercepted return of 0x08006300; sink 0x0800d414 not simulated.
 * Sink is assumed not to mutate these fields in this boundary model. */
#include "recovered_models.h"
void recovered_sub_target(target_state *s,float sensor,int mode,int32_t v,int16_t w,uint32_t arg) {
    if((double)sensor>0.3 && (double)sensor<1.5 && v>0)return;
    if(mode==1) {
        s->previous_v=s->previous_v>80?s->previous_v-50:(s->previous_v< -80?s->previous_v+50:0);
        s->previous_w=s->previous_w>50?s->previous_w-50:(s->previous_w< -50?s->previous_w+50:0);
        s->target_v=s->previous_v;s->target_w=s->previous_w;
        s->calls++;s->argument=65535;
    } else {
        s->target_v=(int16_t)v;s->target_w=w;
        s->calls++;s->argument=arg;
        s->previous_v=(int16_t)v;s->previous_w=w;
    }
}
