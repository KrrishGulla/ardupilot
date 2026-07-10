#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include "AP_RangeFinder_JIYI_CAN.h"
#include <AP_HAL/utility/sparse-endian.h>
#include <GCS_MAVLink/GCS.h>

#if AP_RANGEFINDER_JIYI_CAN_ENABLED

/*
  Handle an incoming CAN frame from a JIYI Microbrain radar sensor.

  Protocol confirmed from captured CSV data:
    Frame type : Standard 11-bit CAN frame
    CAN ID     : 0x00D6
    DLC        : 8 bytes
    Byte 0-1   : Magic word 0xEA2D (big-endian)
    Byte 2-3   : Message type 0x0400 (big-endian)
    Byte 4-5   : Distance in cm (big-endian uint16, 0x0000 = no target)
    Byte 6-7   : SNR/checksum (not used)
*/
bool AP_RangeFinder_JIYI_CAN::handle_frame(AP_HAL::CANFrame &frame)
{
    WITH_SEMAPHORE(_sem);
// DEBUG - remove after testing
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "JIYI: frame received id=0x%03X dlc=%d", 
                  (unsigned)(frame.id & 0x7FF), (int)frame.dlc);
    // JIYI uses standard 11-bit frames only
    if (frame.isExtended()) {
        return false;
    }

    // Filter by CAN ID 0x00D6
    const uint16_t id = frame.id & AP_HAL::CANFrame::MaskStdID;
    if (id != JIYI_CAN_ID) {
        return false;
    }

    // Must be exactly 8 bytes
    if (frame.dlc != 8) {
        return false;
    }

    // Bytes 0-1: magic word (big-endian)
    const uint16_t magic = be16toh_ptr(&frame.data[0]);
    if (magic != JIYI_MAGIC) {
        return false;
    }

    // Bytes 2-3: message type (big-endian)
    const uint16_t msg_type = be16toh_ptr(&frame.data[2]);
    if (msg_type != JIYI_MSG_DIST) {
        return true;
    }

    // Bytes 4-5: distance in cm (big-endian uint16)
    const uint16_t dist_cm = be16toh_ptr(&frame.data[4]);

    if (dist_cm == 0) {
        return true;
    }

    accumulate_distance_m(dist_cm * 0.01f);

    return true;
}

#endif  // AP_RANGEFINDER_JIYI_CAN_ENABLED