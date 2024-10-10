#include <iostream> // header in standard library

namespace Jumping 
{
    class Motor_69
    {
    public:
        Motor_69();
        ~Motor_69();

        /*
        * Description:  pulling the piston to the top
        * Intput: 
        * Output: success=0
        */
        int accelerate();
        
        /*
        * Description:  Keeping the piston position on top
        * Intput: 
        * Output: success=0
        */
        int keep_postion();

    private:
        float MAX_CURRENT = 0;
    };
}