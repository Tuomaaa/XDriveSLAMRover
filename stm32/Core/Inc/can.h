#ifndef CAN_H
#define CAN_H
#include <stdint.h>

/* CAN frame flags (from Linux SocketCAN) */
#define CAN_EFF_FLAG  0x80000000U   /* extended frame format */
#define CAN_RTR_FLAG  0x40000000U   /* remote transmission request */
#define CAN_ERR_FLAG  0x20000000U   /* error message frame */

#define CAN_SFF_MASK  0x000007FFU   /* standard frame ID mask (11 bits) */
#define CAN_EFF_MASK  0x1FFFFFFFU   /* extended frame ID mask (29 bits) */

#define CAN_MAX_DLEN  8             /* max data length */

typedef struct {
    uint32_t can_id;
    uint8_t  can_dlc;
    uint8_t  data[8];
} can_frame;

#endif