#include "AP_Proximity_config.h"

#if AP_PROXIMITY_JIYI_CAN_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include "AP_Proximity_JIYI_CAN.h"

const AP_Param::GroupInfo AP_Proximity_JIYI_CAN::var_info[] = {
    AP_GROUPEND
};

AP_Proximity_JIYI_CAN::AP_Proximity_JIYI_CAN(AP_Proximity &_frontend,
                                              AP_Proximity::Proximity_State &_state,
                                              AP_Proximity_Params &_params)
    : AP_Proximity_Backend(_frontend, _state, _params)
{
    multican_JIYI = NEW_NOTHROW MultiCAN{
        FUNCTOR_BIND_MEMBER(&AP_Proximity_JIYI_CAN::handle_frame, bool, AP_HAL::CANFrame &),
        AP_CAN::Protocol::JIYI,
        "JIYI MultiCAN"
    };
    if (multican_JIYI == nullptr) {
        AP_BoardConfig::allocation_error("Failed to create JIYI proximity multican");
    }

    AP_Param::setup_object_defaults(this, var_info);
    state.var_info = var_info;
}

void AP_Proximity_JIYI_CAN::update(void)
{
    WITH_SEMAPHORE(_sem);
    const uint32_t now = AP_HAL::millis();
    if (now - last_update_ms > 500) {
        set_status(AP_Proximity::Status::NoData);
    } else {
        set_status(AP_Proximity::Status::Good);
    }
}

bool AP_Proximity_JIYI_CAN::handle_frame(AP_HAL::CANFrame &frame)
{
    WITH_SEMAPHORE(_sem);

    // R21-1 uses 29-bit extended frames only
    if (!frame.isExtended()) {
        return false;
    }

    // Filter by CAN ID — accept front (0x73C) and rear (0x74C)
    const uint32_t id = frame.id & AP_HAL::CANFrame::MaskExtID;
    if (id != JIYI_R21_CAN_ID_FRONT && id != JIYI_R21_CAN_ID_REAR) {
        return false;
    }

    // R21-1 sends 6 bytes (3 targets x 2 bytes each)
    if (frame.dlc != 6) {
        return false;
    }

    // Read all 3 targets (big-endian uint16, cm each)
    const uint16_t t1 = (uint16_t(frame.data[0]) << 8) | frame.data[1];
    const uint16_t t2 = (uint16_t(frame.data[2]) << 8) | frame.data[3];
    const uint16_t t3 = (uint16_t(frame.data[4]) << 8) | frame.data[5];

    // Find minimum non-zero target (closest obstacle)
    uint16_t dist_cm = 0;
    if (t1 > 0) dist_cm = t1;
    if (t2 > 0 && (dist_cm == 0 || t2 < dist_cm)) dist_cm = t2;
    if (t3 > 0 && (dist_cm == 0 || t3 < dist_cm)) dist_cm = t3;

    if (dist_cm == 0) {
        return true;
    }

    const float dist_m = dist_cm * 0.01f;

    if (dist_m < JIYI_R21_MIN_RANGE_M || dist_m > JIYI_R21_MAX_RANGE_M) {
        return true;
    }

    // yaw = 0 means forward (orientation handled by PRX1_ORIENT param)
    const float yaw = correct_angle_for_orientation(0.0f);

    if (!ignore_reading(yaw, dist_m)) {
        const AP_Proximity_Boundary_3D::Face face = frontend.boundary.get_face(yaw);
        _temp_boundary.add_distance(face, yaw, dist_m);
        _temp_boundary.update_3D_boundary(state.instance, frontend.boundary);
        database_push(yaw, dist_m);
        last_update_ms = AP_HAL::millis();
    }

    return true;
}

#endif  // AP_PROXIMITY_JIYI_CAN_ENABLED