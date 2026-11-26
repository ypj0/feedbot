/* getdatafromdriver 0x47830 normal path, pure packet/state boundary.
 * smooth_v is low16 returned by smooth_pwm_auto; angular result is discarded.
 * No serial I/O, exceptions, blocking receive, concurrent state or hardware.
 * out[40], state at least 0xb00; finite in-range doubles at 398/3a0/3a8. */
#include "recovered_models.h"
#include <string.h>
static uint16_t u16(const uint8_t *p){uint16_t x;memcpy(&x,p,2);return x;}
static int16_t i16(const uint8_t *p){int16_t x;memcpy(&x,p,2);return x;}
static uint32_t u32(const uint8_t *p){uint32_t x;memcpy(&x,p,4);return x;}
static uint64_t u64(const uint8_t *p){uint64_t x;memcpy(&x,p,8);return x;}
static double dbl(const uint8_t *p){double x;memcpy(&x,p,8);return x;}
static void be16(uint8_t *p,uint16_t x){p[0]=(uint8_t)(x>>8);p[1]=(uint8_t)x;}
int recovered_driver_frame(uint8_t *s,int msg,int16_t smooth_v,uint8_t out[40]) {
    memset(out,0,40);
    if(msg<1 || msg>3)return 0;
    out[0]=out[1]=127;out[2]=40;out[3]=(uint8_t)msg;out[38]=13;out[39]=10;
    if(msg==3){memcpy(out+4,s+0x758,4);memset(s+0x758,0,4);}
    if(msg==2){
        if(s[0x718]==s[0x738]){
            out[7]=s[0x71c];s[0x71c]=0;out[8]=s[0x71d];out[9]=s[0xafa];
            int v=smooth_v;if(v< -600)v=-600;if(v>1500)v=1500;
            int w=i16(s+0x724);if(w>1000)w=1000;if(w< -1000)w=-1000;
            int16_t w16=(int16_t)w;memcpy(s+0x724,&w16,2);
            be16(out+10,(uint16_t)v);be16(out+12,(uint16_t)w);
        } else {
            out[4]=s[0x718];s[0x718]=255;be16(out+5,u16(s+0x71a));
            out[7]=s[0x71c];s[0x71c]=0;out[8]=s[0x71d];out[9]=s[0xafa];
        }
        if((u64(s+0x720)&UINT64_C(0xffffffff0000))==0){
            if(u64(s+0x578)==0){s[0x40b]=s[0x409]=0;}
            else {s[0x40b]++;if(s[0x40b]>100)s[0x409]=1;}
        } else if(u64(s+0x578)==0){s[0x40a]++;if(s[0x40a]>100)s[0x408]=1;}
        else s[0x40a]=s[0x408]=0;
        be16(out+14,u16(s+0x726));be16(out+16,u16(s+0x728));
        if(s[0x72a]==1){out[18]=1;s[0x72a]=0;}
        be16(out+19,u16(s+0x720));out[21]=s[0x72b];out[22]=s[0x71e];
        out[23]=(uint8_t)(int32_t)dbl(s+0x398);out[24]=(uint8_t)(int32_t)dbl(s+0x3a0);out[25]=(uint8_t)(int32_t)dbl(s+0x3a8);
        out[26]=s[0x730];uint32_t field=u32(s+0x72c);out[34]=(uint8_t)(field>>24);out[35]=(uint8_t)(field>>16);out[27]=(uint8_t)(field>>8);out[28]=(uint8_t)field;
        be16(out+29,u16(s+0x732));out[31]=s[0x734];out[32]=s[0xaf4];out[33]=s[0xaf6];out[36]=s[0xaf8];
        memcpy(s+0x738,s+0x718,32);
        memset(s+0xaf6,0,6);
    }
    uint8_t sum=0;for(int i=0;i<37;i++)sum=(uint8_t)(sum+out[i]);out[37]=(uint8_t)(0-sum);
    return 1;
}
