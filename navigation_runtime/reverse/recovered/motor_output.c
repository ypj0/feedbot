/* Offline semantic recovery: sub.hex 0x0800b0e8 -> 0x0800ab00 -> 0x0800ab84.
 * No CAN/device access. Node identity/units require board validation.
 */
#include "recovered_models.h"
#include <string.h>
static uint16_t get16(const uint8_t *p){return (uint16_t)(p[0]|((uint16_t)p[1]<<8));}
static uint32_t get32(const uint8_t *p){return p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
static void put16(uint8_t *p,uint16_t v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
static void put32(uint8_t *p,uint32_t v){for(unsigned i=0;i<4;i++)p[i]=(uint8_t)(v>>(8*i));}
static uint32_t sign16(uint16_t v){return v&0x8000?0xffff0000u|v:v;}
/* 0x0800ab84: 8-byte expedited write, CANopen-shaped 0x60ff:00 target. */
void recovered_motor_frame(uint32_t node,int32_t target,MotorFrame *out){
 out->id=node+0x600u;out->data[0]=0x23;out->data[1]=0xff;
 out->data[2]=0x60;out->data[3]=0;put32(out->data+4,(uint32_t)target);
}
/* 0x0800b0e8 / 0x0800ab00: register block base 0x20001ea4, flags b5/b6. */
void recovered_motor_commit(uint8_t state[0x48],uint8_t flags[2],
 int32_t target_a,int32_t target_b,uint32_t span,MotorOutput *out){
 out->count=0;
 put32(state+4,span);
 memcpy(state+8,state+0x2c,4);memcpy(state+0xc,state+0x44,4);
 memcpy(state+0x14,state+0x10,4);
 put16(state+0x10,(uint16_t)target_a);
 put16(state+0x12,(uint16_t)(0u-(uint32_t)target_b));
 uint32_t a=sign16(get16(state+0x10)),b=sign16(get16(state+0x12));
 /* ARM MUL wraps at 32 bits; do not use signed-overflow C arithmetic. */
 if((get32(state+0x18)*a)&0x80000000u)flags[0]=flags[1]=1;
 else if((get32(state+0x1c)*b)&0x80000000u)flags[0]=flags[1]=1;
 if(state[0x3c]==0x37)recovered_motor_frame(2,(int32_t)b,&out->frames[out->count++]);
 if(state[0x24]==0x37)recovered_motor_frame(1,(int32_t)a,&out->frames[out->count++]);
}
