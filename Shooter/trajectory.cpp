#include "trajectory.h"
// #include <pybind11/pybind11.h>
// #include <pybind11/eigen.h>

// namespace py = pybind11;

// PYBIND11_MODULE(MyLib, m) {
//     m.doc() = "trajectory";

//     py::class_<MyClass>(m, "trajectory")
//     .def(py::init<double, double, int>())  
//     .def("calculate_velocity", &trajectory::calculate_velocity, py::call_guard<py::gil_scoped_release>())
    
//     ;
// }

double trajectory::calculate_velocity(double D, double H_shooter, double angle) {
    double velocity = 0;
    double height_diff = H_RIM - H_shooter;

    double radians = angle * M_PI / 180.0;
    //trajectory equation: v_0 = sqrt((g * x^2) / (2 * (x * tan(theta) - y) * cos^2(theta)))
    velocity = sqrt((g * pow(D, 2)) / (2 * (D * tan(radians) - height_diff) * cos(radians) * cos(radians)));
    return velocity;
}

double trajectory::calculate_angle(double D, double H_shooter, double velocity) {
    double height_diff = H_RIM - H_shooter;

    // Calculate initial velocity using a range of angles
    double bestAngle = -1;
    double min_error = std::numeric_limits<double>::max();
    
    for (double theta = 1; theta <= 90; theta += 0.1) { // Iterate over angles from 1 degree to 45 degree
        
        double radians = theta * M_PI / 180.0;

        //trajectory equation: y = x\tan{\theta} - \frac{gx^2}{2v_0^2\cos^2{\theta}}
        double height = D * tan(radians) - (g * pow(D, 2)) / (2 * pow(velocity, 2) * pow(cos(radians), 2));

        //double radians = theta * M_PI / 180.0; // Convert angle to radians
        //double v0 = sqrt((g * D * D) / (2 * (D * tan(radians) + hight_diff)));
        double error = fabs(height - height_diff);

        if (fabs(height - height_diff) <= ERROR_TOLERANCE && error < min_error) {
            bestAngle = theta;
            min_error = error;
        }
    }

    return bestAngle;
}

void trajectory::calculateTrajectory(double initialVelocity, double launchAngle) {
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

