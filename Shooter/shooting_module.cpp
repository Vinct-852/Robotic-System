#include "shooting_module.h"

Shooting_module::Shooting_module(CAN* can, std::vector<int> can_id_list)
{
    // Check if can_id_list has at least 5 elements
    if (can_id_list.size() < 5) {
        throw std::runtime_error("Error: can_id_list must contain at least 5 elements.");
    }

    left_1_motor = vesc_control(can, can_id_list[0]);
    left_2_motor = vesc_control(can, can_id_list[1]);
    middle_motor = vesc_control(can, can_id_list[2]);
    right_1_motor = vesc_control(can, can_id_list[3]);
    right_2_motor = vesc_control(can, can_id_list[4]);
}

int Shooting_module::set_all_shooter_rmp(float rpm)
{
    try
    {
        middle_motor.set_rpm(rpm);
        left_1_motor.set_rpm(rpm);
        left_2_motor.set_rpm(rpm);
        right_1_motor.set_rpm(rpm);
        right_2_motor.set_rpm(rpm);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return -1;
    }

    return 0;
}

int Shooting_module::fixed_angle_shoot(double distant, double shooter_height, double angle)
{
    try
    {
        double velocity = calculate_velocity(distant, shooter_height, angle)/SHOOTING_MOTORS_NUMBER;
        // velocity to rpm (V / (π × D)) × 60
        float rpm = (velocity/(M_PI*SHOOTING_MOTORS_DIAMETOR))*60;

        int res set_all_shooter_rmp(rpm);
        if(res != 0){
            printf("set_all_shooter_rmp failed\n");
            return -1;
        }
    }   
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return -1;
    }
    
    return 0;
}
