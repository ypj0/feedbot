/* 0x0800d414, offline recovered kinematics + its local gates.
 * Offset names are structural, not proven physical calibration units.
 * Finite arithmetic, nonzero byte7 and intermediate signed conversion in range.
 * External 0x080062bc is supplied; 0x0800b0e8 is recorded, no peripherals. */
#include "recovered_models.h"
#include <string.h>
static uint16_t u16(const unsigned char *p){uint16_t x;memcpy(&x,p,2);return x;}
static int16_t i16(const unsigned char *p){int16_t x;memcpy(&x,p,2);return x;}
static uint32_t u32(const unsigned char *p){uint32_t x;memcpy(&x,p,4);return x;}
static void set16(unsigned char *p,uint16_t x){memcpy(p,&x,2);}
static int32_t low16(double x){uint16_t a=(uint16_t)(int32_t)x;int16_t b;memcpy(&b,&a,2);return b;}
void recovered_sub_kinematics(unsigned char *s,uint8_t mode,int gate,
                             const uint16_t near[3],uint32_t arg,int32_t out[3]) {
    if(mode==2 && u16(s)==1) {
        if(i16(s+16)<=0)set16(s,0);
        if(u16(s)==1)set16(s+16,(uint16_t)(u16(s+16)-1));
        if(i16(s+2)>0 && u16(s+4)==0 && (near[0]<30 || near[1]<20 || near[2]<30)) {
            set16(s,0);set16(s+16,0);out[0]=out[1]=out[2]=0;return;
        }
    }
    if(gate<=0){set16(s,0);set16(s+16,0);}
    double scale=((double)s[14]*60000.0)/((double)s[7]*3.1415926);
    float angular_product=(float)i16(s+4)*(float)u16(s+12);
    double turn=(double)angular_product/2000.0;
    double v=(double)(float)i16(s+2);
    out[0]=low16(((v-turn)/1000.0)*scale);
    out[1]=low16(((v+turn)/1000.0)*scale);
    if(u16(s+2)!=0)
        out[2]=(int32_t)((((double)(float)arg/((double)s[7]*3.1415926))*(double)u32(s+8))*10.0);
    else if(u16(s+4)!=0) {
        float product=(float)arg*(float)u16(s+12);
        out[2]=(int32_t)((((double)product/((double)s[7]*12566.3704))*(double)u32(s+8))*10.0);
    } else out[2]=0;
}
