#include "jumping_module.h"

int jumping_motor::set_position(float pos){

    try
    {
        set_pos(pos);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return -1;
    }
    
    std::cout << "Motor " << can_id << " position set to: " << pos << std::endl;
    return 0;
}
