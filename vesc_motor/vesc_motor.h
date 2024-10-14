#include "vesc_control.h"


class vesc_motor {
    public:
    vesc_motor(CAN* can, int can_id);
    ~vesc_motor();

    private:
    int can_id;
    vesc_control* motor_driver;
};