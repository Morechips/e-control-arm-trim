#ifndef TEST_PHONE20_PACKET_H
#define TEST_PHONE20_PACKET_H
#include <stdint.h>
/* Schema read from the user's (20).pro; values are signed LE shorts.
 * JOY_Y,F,B,STOP,L,R,R90,R180,Cam_T,Shot,L90,servo_mode,joy_x,armcmd,armx,army. */
static void PackPhone20(uint8_t p[41], const int16_t values[16],
                        uint16_t buttons, uint32_t gap)
{
    unsigned i;
    p[0] = 0xA5U;
    p[1] = (uint8_t)buttons;
    p[2] = (uint8_t)(buttons >> 8);
    for (i = 0U; i < 16U; ++i) {
        p[3U + 2U*i] = (uint8_t)values[i];
        p[4U + 2U*i] = (uint8_t)((uint16_t)values[i] >> 8);
    }
    for (i = 0U; i < 4U; ++i) p[35U+i] = (uint8_t)(gap >> (8U*i));
    p[39] = 0U;
    for (i = 1U; i <= 38U; ++i) p[39] = (uint8_t)(p[39] + p[i]);
    p[40] = 0x5AU;
}
#endif
