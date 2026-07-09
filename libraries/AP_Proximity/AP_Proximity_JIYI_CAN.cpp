#include "AP_Proximity_config.h"

#if AP_PROXIMITY_JIYI_CAN_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include "AP_Proximity_JIYI_CAN.h"

const AP_Param::GroupInfo AP_Proximity_JIYI_CAN::var_info[] = {

    // @Param: RECV_ID
    // @DisplayName: CAN receive ID
    // @Description: The receive ID of the CAN frames. Zero means all IDs accepted.
    // @Range: 0 65535
    // @User: Advanced
    AP_GROUPINFO("RECV_ID", 1, AP_Proximity_JIYI_CAN, receive_id, 0),

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

    // reject extended frames
    if (frame.isExtended()) {
        return false;
    }

    // check receive ID filter
    const uint16_t id = frame.id & AP_HAL::CANFrame::MaskStdID;
    if (receive_id > 0 && id != uint32_t(receive_id)) {
        return false;
    }

    // must be 8 bytes
    if (frame.dlc != 8) {
        return false;
    }

    // bytes 0-1: magic word
    const uint16_t magic = be16toh_ptr(&frame.data[0]);
    if (magic != JIYI_MAGIC) {
        return false;
    }

    // bytes 2-3: message type
    const uint16_t msg_type = be16toh_ptr(&frame.data[2]);
    if (msg_type != JIYI_MSG_DIST) {
        return true;  // consumed but not a distance frame
    }

    // bytes 4-5: distance in cm
    const uint16_t dist_cm = be16toh_ptr(&frame.data[4]);
    if (dist_cm == JIYI_NO_TARGET) {
        return true;
    }

    const float dist_m = dist_cm * 0.01f;

    if (dist_m < JIYI_MIN_RANGE_M || dist_m > JIYI_MAX_RANGE_M) {
        return true;
    }

    // yaw angle comes from the sensor orientation set in params
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