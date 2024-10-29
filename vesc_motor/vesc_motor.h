#include "vesc_control.h"


class vesc_motor {
    public:
    vesc_motor(CAN* can, int can_id);
    ~vesc_motor();

    //base functions
    void set_rpm(float rpm);
    void set_pos(float pos);

    //customized functions


    private:
    int can_id;
    vesc_control* motor_driver;
};