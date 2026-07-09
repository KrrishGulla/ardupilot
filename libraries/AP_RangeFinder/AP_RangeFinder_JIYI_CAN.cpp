#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include "AP_RangeFinder_JIYI_CAN.h"

#if AP_RANGEFINDER_JIYI_CAN_ENABLED

/*
  Handle an incoming CAN frame from a JIYI UAV-H30-1 altitude radar.

  CAN frame format:
    Frame type : CAN2.0A 29-bit extended frame
    CAN ID     : 0x75C
    DLC        : 6 bytes (3 targets x 2 bytes each)
    Byte 0-1   : Target1 distance, big-endian uint16, cm (0 = no target)
    Byte 2-3   : Target2 distance, big-endian uint16, cm (0 = no target)
    Byte 4-5   : Target3 distance, big-endian uint16, cm (0 = no target)
*/
bool AP_RangeFinder_JIYI_CAN::handle_frame(AP_HAL::CANFrame &frame)
{
    WITH_SEMAPHORE(_sem);

    // H30-1 uses 29-bit extended frames only
    if (!frame.isExtended()) {
        return false;
    }

    // Filter by H30-1 CAN ID: 0x75C
    const uint32_t id = frame.id & AP_HAL::CANFrame::MaskExtID;
    if (id != JIYI_H30_CAN_ID) {
        return false;
    }

    // Must be exactly 6 bytes
    if (frame.dlc != 6) {
        return false;
    }

    // Read all 3 targets (big-endian uint16, cm each)
    const uint16_t t1 = (uint16_t(frame.data[0]) << 8) | frame.data[1];
    const uint16_t t2 = (uint16_t(frame.data[2]) << 8) | frame.data[3];
    const uint16_t t3 = (uint16_t(frame.data[4]) << 8) | frame.data[5];

    // Find minimum non-zero target (closest valid ground return)
    uint16_t dist_cm = 0;
    if (t1 > 0) dist_cm = t1;
    if (t2 > 0 && (dist_cm == 0 || t2 < dist_cm)) dist_cm = t2;
    if (t3 > 0 && (dist_cm == 0 || t3 < dist_cm)) dist_cm = t3;

    if (dist_cm == 0) {
        // no target detected
        return true;
    }

    accumulate_distance_m(dist_cm * 0.01f);

    return true;
}

#endif  // AP_RANGEFINDER_JIYI_CAN_ENABLED