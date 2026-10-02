#include "detect_data.h"
#include <assert.h>
#include <stddef.h>
#include <stdio.h>

int main(void)
{
    DetectData data = {0};

    assert(QRCODE_NOTIFY_PACK == 0x80U);
    assert(MODE_CMD_AIM == 0x00U);
    assert(MODE_CMD_OBJECT == 0x01U);
    assert(DETECT_REDBALL == 0);
    assert(DETECT_BLUEBALL == 1);
    assert(DETECT_GREANBALL == 2);
    assert(DETECT_HOSTAGE_1 == 3);
    assert(DETECT_HOSTAGE_2 == 4);
    assert(DETECT_HOSTAGE_3 == 5);
    assert(DETECT_BARREL == 6);
    assert(DETECT_TARGET == 7);
    assert(DETECT_UNKNOWN == 8);

    assert(sizeof(DetectData) == 5U);
    assert(offsetof(DetectData, type) == 0U);
    assert(offsetof(DetectData, offset_x) == 1U);
    assert(offsetof(DetectData, offset_y) == 3U);

    data.type = DETECT_HOSTAGE_2;
    data.offset_x = -123;
    data.offset_y = 456;
    assert(data.type == DETECT_HOSTAGE_2);
    assert(data.offset_x == -123);
    assert(data.offset_y == 456);

    puts("PASS MaxiCam protocol: binary QR 0x80 / packed 5-byte position data");
    return 0;
}
