#include <iostream>
#include <cmath>
#include <iomanip>
#include <vector>
#include <float.h>


/*
1.  The best way to shoot free throws depends upon the person
    shooting. The two most important factors are their height and
    how consistent they are in controlling both the release angle and
    the release velocity.

2.  In general, the taller you are, the lower your release angle should
    be. We’ll actually see that taller players are allowed more error
    in both their release angles and release velocities and thus they
    should have an easier time shooting free throws than shorter players.

3.  It is much more important to consistently use the right release
    elocity than the right release angle.

4.  The best shot does not pass through the center of the hoop. The best
    trajectories pass through the hoop somewhere between the center
    and the back rim. Taller players should shoot closer to the center
    while shorter players should aim more towards the back rim.
*/

#define g 9.81 // Acceleration due to gravity (m/s^2)
#define H_RIM 2.43 // The height of the hoop (mm)
#define M_PI 3.14159265358979323846 // Define M_PI manually
#define ERROR_TOLERANCE 2 

class trajectory {
public:
    
    /*
        Calulate the required velocity by distant, height and angle
    */
    double calculate_velocity(double D, double H_shooter, double angle);

    /*
        Calulate the best angle by distant, height and velocity
    */
    double calculate_angle(double D, double H_shooter, double velocity);

    // Generated using AI
    void calculateTrajectory(double initialVelocity, double launchAngle);
    
private:
    
};

extern "C" {
    trajectory* create_trajectory() {
        return new trajectory();
    }

    void destroy_trajectory(trajectory* t) {
        delete t;
    }

    double calculate_velocity(trajectory* t, double D, double H_shooter, double angle) {
        return t->calculate_velocity(D, H_shooter, angle);
    }

    double calculate_angle(trajectory* t, double D, double H_shooter, double velocity) {
        return t->calculate_angle(D, H_shooter, velocity);
    }
}