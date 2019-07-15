#include <iostream>
#include <iomanip>
#include "robotics.hpp"
#include "Mat.hpp"

using namespace std;
using namespace rrt;

#define STAND_IN_SENSOR(output_value) new double(output_value)

int main()
{
    D_H_Table dh;
    unique_ptr<double> encoder (STAND_IN_SENSOR(50));         // Pointers are used to allow the D_H_Table to access the memory
    unique_ptr<double> distance (STAND_IN_SENSOR(6));         // without having to reinitialize
    double angle_factor = 0.01234; // rad/tick 
    double distance_factor = 0.1234; // in/tick

    dh.addFrame( 0,       0,               0,       0,
                 0,       0,  distance.get(),       0,
                 1,       1, distance_factor,       1); // Elevator

    dh.addFrame( -pi/2,       0,      20,       0,
         encoder.get(),       0,       0,       0,
          angle_factor,       1,       1,       1);   // Semigorgan

    cout << dh.getH_Matrix(4) << "\n\n";      // Prints matrix that describes the position and orientation of the Semigorgan

    *encoder = 100;                           // Move the maipulator

    cout << dh.getH_Matrix(4) << "\n\n";      // Prints matrix that describes the position and orientation of the Semigorgan

    return 0;
}