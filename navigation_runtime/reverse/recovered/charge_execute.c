/* ChargeController::execute 0x1066a0, explicit external-effect boundary.
 * Offline model. Callback implements perception/control/time/publish effects.
 * Pose copies, allocation, ROS exceptions and log side effects are not modeled.
 * op: 1 found, 2 timeout predicate, 3 approach, 4 final, 5 stop perception,
 *     6 publish charge_fail(arg), 7 publish navigation_status(arg).
 */
#include "recovered_models.h"
#include <string.h>

static void zero_command(unsigned char *t) {
    memset(t,0,16);memset(t+40,0,8);
}
int recovered_charge_execute(unsigned char *s,unsigned char *t,
                             charge_effect effect,void *ctx) {
    if(!s[8]) {zero_command(t);effect(5,0,ctx);}
    else {
        if(s[0x6e8]) {
            effect(6,0,ctx);zero_command(t);effect(5,0,ctx);
            s[0x67c]=86;effect(7,86,ctx);return 4;
        }
        if(effect(1,0,ctx)==1) {
            if(!effect(2,0,ctx))return 0;
            zero_command(t);s[0x6f0]=1;
        }
        int failed=0;
        if(!s[0x748]) {
            int r=effect(3,0,ctx);
            if(r==0) {s[0x748]=1;effect(5,0,ctx);}
            else if(r==-1)failed=1;
            if(!failed && !s[0x748])return 0;
        }
        if(!failed && effect(4,0,ctx))return 0;
        effect(5,0,ctx);effect(6,5,ctx);
    }
    s[0x67c]=87;effect(7,87,ctx);return 1;
}
