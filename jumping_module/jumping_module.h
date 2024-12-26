#include "jumping_motor.h"

#define MIN_POS 0

class jumping_module {

    jumping_motor* left_motor;
    jumping_motor* right_motor;

    public:
        jumping_module(jumping_motor* left_motor, jumping_motor* right_motor);
        int set_all_position(float pos);
};