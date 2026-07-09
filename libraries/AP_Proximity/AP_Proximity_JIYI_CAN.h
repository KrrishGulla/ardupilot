#pragma once
#include "AP_Proximity_config.h"

#if AP_PROXIMITY_JIYI_CAN_ENABLED

#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANSensor.h>

#define JIYI_MAX_RANGE_M    27.0f   // UAV-R21-1 max range in meters
#define JIYI_MIN_RANGE_M     1.5f   // UAV-R21-1 min range in meters

class JIYI_MultiCAN;

class AP_Proximity_JIYI_CAN : public AP_Proximity_Backend {
public:
    friend class JIYI_MultiCAN;

    AP_Proximity_JIYI_CAN(AP_Proximity &_frontend,
                          AP_Proximity::Proximity_State &_state,
                          AP_Proximity_Params &_params);

    void update() override;

    bool handle_frame(AP_HAL::CANFrame &frame);

    float distance_max_m() const override { return JIYI_MAX_RANGE_M; }
    float distance_min_m() const override { return JIYI_MIN_RANGE_M; }

    static const struct AP_Param::GroupInfo var_info[];

private:
    static constexpr uint16_t JIYI_MAGIC     = 0xEA2D;
    static constexpr uint16_t JIYI_MSG_DIST  = 0x0400;
    static constexpr uint16_t JIYI_NO_TARGET = 0x0000;

    uint32_t last_update_ms;
    AP_Int32 receive_id;

    MultiCAN *multican_JIYI;
    AP_Proximity_Temp_Boundary _temp_boundary;
};

#endif  // AP_PROXIMITY_JIYI_CAN_ENABLED