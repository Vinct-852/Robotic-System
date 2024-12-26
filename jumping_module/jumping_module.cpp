#include "jumping_module.h"

jumping_module::jumping_module(jumping_motor* left_motor, jumping_motor* right_motor)
{
    this->left_motor = left_motor;
    this->right_motor = right_motor;
}

int jumping_module::set_all_position(float pos){
    if(pos < MIN_POS) return -1;

    try
    {
        left_motor->set_position(pos);
        right_motor->set_position(pos);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return -1;
    }
    
    return 0;
}