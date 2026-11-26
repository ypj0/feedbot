/* Offline recovery: ltme_node 0x4dfb0 OOB checksum validation.
 * Nonnegative length, accessible buffer of at least length bytes.
 * Packet framing/checksum only: not sufficient payload-length validation. */
#include "recovered_models.h"
#include <stddef.h>
uint16_t recovered_lidar_crc(const uint8_t *data,size_t length) {
    uint16_t crc=0xffff;
    for(size_t i=0;i<length;i++) {
        uint8_t v=(uint8_t)((crc>>8)^data[i]);v^=v>>4;
        crc=(uint16_t)((crc<<8)^((uint16_t)v<<12)^((uint16_t)v<<5)^v);
    }
    return crc;
}
int recovered_lidar_verify(uint8_t *data,int length) {
    if(length<16 || data[0]!=255 || data[1]!=255)return 0;
    uint16_t saved=(uint16_t)(data[12]|((uint16_t)data[13]<<8));
    data[12]=data[13]=0;
    uint16_t crc=recovered_lidar_crc(data,(size_t)length);
    data[12]=(uint8_t)saved;data[13]=(uint8_t)(saved>>8);
    return crc==saved;
}
