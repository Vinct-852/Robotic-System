#include "vesc_control.h"
#include <vector>

class shooting_module {

    vesc_control left_1_motor;
    vesc_control left_2_motor;

    vesc_control right_1_motor;
    vesc_control right_2_motor;

    vesc_control middle_motor;

    public:
    shooting_module(CAN* can, std::vector<int> can_id_list);
    ~shooting_module();

    int set_shooter_rmp(float rpm);
};