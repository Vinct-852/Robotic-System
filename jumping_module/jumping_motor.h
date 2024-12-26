#include "vesc_control.h"

class jumping_motor : vesc_control {
    public:
        jumping_motor(CAN* can, int can_id) : vesc_control(can, can_id){}

        int set_position(float pos);
};