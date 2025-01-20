#include "vesc_control.h"
#include "trajectory.h"

#include <vector>

#define SHOOTING_MOTORS_DIAMETOR 0.04
#define SHOOTING_MOTORS_NUMBER 5

class Shooting_module {

    vesc_control left_1_motor;
    vesc_control left_2_motor;

    vesc_control right_1_motor;
    vesc_control right_2_motor;

    vesc_control middle_motor;

public:
    Shooting_module(CAN* can, std::vector<int> can_id_list);

    int set_all_shooter_rmp(float rpm);
    int fixed_angle_shoot(double distant, double shooter_height, double launch_angle);
};

/*
//simple code

vector<int> shooting_motors_can_id_list = {1, 2, 3, 4, 5};
Shooting_module shooting_module(can, shooting_motors_can_id_list);

*/