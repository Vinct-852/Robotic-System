#include <iostream>
#include <cmath>
#include <iomanip>
#include <vector>
#include <corecrt_math_defines.h>
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
#define H_RIM 2430 // The height of the hoop (mm)
#define ERROR_TOLERANCE 2 

class trajectory {
public:
    
    /*
        Calulate the required velocity by distant, height and angle
    */
    double calculate_velocity(double D, double H_shooter, double angle) {
        double velocity = 0;
        double height_diff = H_RIM - H_shooter;

        //trajectory equation: v_0 = sqrt((g * x^2) / (2 * (x * tan(theta) - y) * cos^2(theta)))
        velocity = sqrt((g*pow(D,2)) / 2*(D*tan(angle)-height_diff)*cos(pow(angle,2)));

        return velocity;
    }

    /*
        Calulate the best angle by distant, height and velocity
    */
    double calculate_angle(double D, double H_shooter, double velocity) {
        double height_diff = H_RIM - H_shooter;

        // Calculate initial velocity using a range of angles
        double bestAngle = 0;
        
        for (double theta = 1; theta < 90; theta += 0.1) { // Iterate over angles from 1° to 89°
            
            //trajectory equation: y = x\tan{\theta} - \frac{gx^2}{2v_0^2\cos^2{\theta}}
            double height = D*tan(theta) - ((g*pow(D,2)) / 2*pow(velocity,2)*cos(pow(theta,2)));

            //double radians = theta * M_PI / 180.0; // Convert angle to radians
            //double v0 = sqrt((g * D * D) / (2 * (D * tan(radians) + hight_diff)));
            
            if (fabs(height - height_diff) <= ERROR_TOLERANCE) {
                bestAngle = theta;
            }
        }

        return bestAngle;
    }

    // Generated using AI
    void calculateTrajectory(double initialVelocity, double launchAngle) {
        // Convert angle from degrees to radians
        double theta = launchAngle * M_PI / 180.0;

        // Calculate total time of flight
        double totalTime = (2 * initialVelocity * sin(theta)) / g;

        // Number of steps for the trajectory
        int steps = 100;
        double timeStep = totalTime / steps;

        std::vector<double> x_vals(steps + 1);
        std::vector<double> y_vals(steps + 1);

        // Calculate trajectory points
        for (int i = 0; i <= steps; ++i) {
            double t = i * timeStep; // Current time
            x_vals[i] = initialVelocity * cos(theta) * t; // Horizontal position
            y_vals[i] = initialVelocity * sin(theta) * t - (0.5 * g * t * t); // Vertical position

            // Output the trajectory points
            std::cout << "t: " << std::fixed << std::setprecision(2) << t << " s, "
                    << "x: " << std::fixed << std::setprecision(2) << x_vals[i] << " m, "
                    << "y: " << std::fixed << std::setprecision(2) << y_vals[i] << " m" << std::endl;
        }
    }
    
private:
    
};