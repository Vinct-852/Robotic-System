#include "vesc_motor.h"

vesc_motor::vesc_motor(CAN *can, int can_id)
{
    this->motor_driver = new vesc_control(can, can_id);
    this->can_id = can_id;
}

vesc_motor::~vesc_motor()
{
    delete this->motor_driver;
}

void vesc_motor::set_rpm(float rpm)
{
    this->motor_driver->set_rpm(rpm);
}

void vesc_motor::set_pos(float pos)
{
    this->motor_driver->set_pos(pos);
}
