#ifndef MECANUM_TEST_H
#define MECANUM_TEST_H
#include "mecanum.h"
typedef enum {
    MEC_STOP = 0, MEC_FORWARD, MEC_BACKWARD, MEC_LEFT, MEC_RIGHT,
    MEC_FORWARD_LEFT, MEC_FORWARD_RIGHT, MEC_BACKWARD_LEFT, MEC_BACKWARD_RIGHT,
    MEC_ACTION_COUNT
} MecanumTestAction_t;
/* Test-build debugger mailbox: write action first, increment request second.
 * Rejected requests are consumed, never replayed after a safety unlock.
 * Test must be driven with live centered Bluetooth packets and enable state. */
extern volatile uint32_t mecanum_test_action, mecanum_test_request;
extern volatile uint8_t mecanum_test_active;
void Mecanum_Test_Init(void);
void Mecanum_Test_Process(uint8_t allowed);
void Mecanum_Test_Vector(MecanumTestAction_t action, int16_t *vx, int16_t *vy, int16_t *omega);
#endif
