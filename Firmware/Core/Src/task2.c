void Task2_Run(void) {
    const float OVERSHOOT_THRESHOLD = 100.0f;
    const float BOX_THRESHOLD = 50.0f;
    const int16_t CONFIRM_COUNT = 20;

    int16_t right_overshoot_count = 0;
    int16_t left_overshoot_count = 0;
    int16_t box_appearing_count = 0;

    // --- Phase 1: Follow right wall until wall ends ---
    float right_distance = Read_ToF(2);
    float right_wall_setpoint = right_distance;

    while (1) {
        right_distance = Read_ToF(2);
        float wall_error = right_distance - right_wall_setpoint;

        if (fabsf(wall_error) <= WALL_DEADBAND) {
            Gait_Step_Forward();
        } else {
            Gait_Turn_Proportional(wall_error);
        }

        if (right_distance > OVERSHOOT_THRESHOLD) {
            right_overshoot_count++;
            if (right_overshoot_count > CONFIRM_COUNT) {
                break; // Move to Phase 2
            }
        } else {
            right_overshoot_count = 0;
        }
    }

    // --- Phase 2: Follow left wall until wall ends ---
    float left_distance = Read_ToF(1);
    float left_wall_setpoint = left_distance;

    while (1) {
        left_distance = Read_ToF(1);
        float wall_error = left_distance - left_wall_setpoint;

        if (fabsf(wall_error) <= WALL_DEADBAND) {
            Gait_Step_Forward();
        } else {
            // Check sign convention: left wall error may need opposite turn sign
            Gait_Turn_Proportional(-wall_error);
        }

        if (left_distance > OVERSHOOT_THRESHOLD) {
            left_overshoot_count++;
            if (left_overshoot_count > CONFIRM_COUNT) {
                break; // Move to Phase 3
            }
        } else {
            left_overshoot_count = 0;
        }
    }

    // --- Phase 3: Advance until box is detected in front ---
    while (1) {
        float front_distance = Read_ToF(0);
        Gait_Step_Forward();

        if (front_distance < BOX_THRESHOLD) {
            box_appearing_count++;
            if (box_appearing_count > CONFIRM_COUNT) {
                break; // Box confirmed; exit Task2_Run
            }
        } else {
            box_appearing_count = 0;
        }
    }
}
