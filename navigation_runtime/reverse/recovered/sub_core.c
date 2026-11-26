/* Semantic recovery of selected sub.hex functions, NOT original source or flashable firmware.
 * SHA256 5baeec4873058a2e0b92c04a979b8acd5bae2062cf47194f06e909fc8e9304df
 * Hardware/external routines are explicit trace boundaries. No hardware I/O.
 */
#include "recovered_models.h"
#include <stddef.h>

static void call(State *s,uint32_t f,uint32_t a,uint32_t b,uint32_t c) {
 if(s->call_count<16) s->calls[s->call_count++]=(Call){f,a,b,c};
}
/* 0x0800c1f8..0x0800c380: entire periodic function; external side effects traced. */
void recovered_periodic(State *s) {
 if((uint32_t)(s->tick-s->last_tick)>1000) s->flag_c2=0;
 s->silence_count=(uint16_t)(s->silence_count+1);
 if(s->silence_count>18) {
  s->silence_count=15;s->flag_c1=0;
  call(s,0x0800d170,0,0,0);call(s,0x0800dee4,0,0,0);
 }
 /* Independent checks in original follow timeout; timeout does NOT return. */
 switch(s->pending_action) {
 case 0x385:call(s,0x0800d170,300,0,5000);break;
 case 0x386:call(s,0x0800d170,(uint32_t)-500,0,5000);break;
 case 0x387:call(s,0x0800d170,0,100,0xc44);break;
 case 0x388:call(s,0x0800d170,0,100,0x1888);break;
 case 0x389:call(s,0x0800d170,0,0,0);break;
 case 0x134:call(s,0x0800e53c,0,0,0);break;
 case 0x135:call(s,0x08008014,0x40010c00,0x4000,0);break;
 case 0x136:call(s,0x08008014,0x40010c00,0x4000,1);break;
 case 0x137:call(s,0x08008014,0x40010c00,0x8000,0);break;
 case 0x138:call(s,0x08008014,0x40010c00,0x8000,1);break;
 case 0x139:
  call(s,0x0800a9cc,1,0,0);call(s,0x08007968,20,0,0);
  call(s,0x0800a9cc,2,0,0);call(s,0x08007968,20,0,0);break;
 default:return;
 }
 s->pending_action=0;
}
/* 0x0800ceb6: caller must supply readable length >=3; model rejects shorter input.
 * Original checks delimiters/checksum but does NOT check byte 2 or family here. */
int recovered_check_frame(const uint8_t *p,size_t n) {
 if(n<3 || n>65535)return 1;
 uint8_t sum=0;
 for(size_t i=0;i<n-3;i++)sum=(uint8_t)(sum+p[i]);
 return !(p[0]==0x7f && p[1]==0x7f && p[n-2]==13 && p[n-1]==10 && p[n-3]==(uint8_t)-sum);
}
/* 0x0800b7fc: tick countdown over 8 entries; fields at offsets 0,2,4,8. */

void recovered_scheduler(Task tasks[8]) {
 for(unsigned i=0;i<8;i++)if(tasks[i].remaining) {
  if(--tasks[i].remaining==0){tasks[i].remaining=tasks[i].period;tasks[i].due=1;}
 }

}
