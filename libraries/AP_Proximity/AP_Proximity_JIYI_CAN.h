#pragma once
#include "AP_Proximity_config.h"

#if AP_PROXIMITY_JIYI_CAN_ENABLED

#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANSensor.h>

#define JIYI_R21_MAX_RANGE_M    27.0f
#define JIYI_R21_MIN_RANGE_M     1.5f

#define JIYI_R21_CAN_ID_FRONT   0x73C
#define JIYI_R21_CAN_ID_REAR    0x74C

class JIYI_MultiCAN;

class AP_Proximity_JIYI_CAN : public AP_Proximity_Backend {
public:
    friend class JIYI_MultiCAN;

    AP_Proximity_JIYI_CAN(AP_Proximity &_frontend,
                          AP_Proximity::Proximity_State &_state,
                          AP_Proximity_Params &_params);

    void update() override;
    bool handle_frame(AP_HAL::CANFrame &frame);

    float distance_max_m() const override { return JIYI_R21_MAX_RANGE_M; }
    float distance_min_m() const override { return JIYI_R21_MIN_RANGE_M; }

    static const struct AP_Param::GroupInfo var_info[];

private:
    uint32_t last_update_ms;
    AP_Proximity_Temp_Boundary _temp_boundary;
    MultiCAN *multican_JIYI;
};

#endif  // AP_PROXIMITY_JIYI_CAN_ENABLED