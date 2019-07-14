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
    double* encoder = STAND_IN_SENSOR(50);         // Pointers are used to allow the D_H_Table to handle the memory
    double* distance = STAND_IN_SENSOR(6);
    double angle_factor = 0.01234; // rad/tick 
    double distance_factor = 0.1234; // in/tick

    dh.addFrame( 0,       0,               0,       0,
           nullptr, nullptr,        distance, nullptr,
                 1,       1, distance_factor,       1); // Elevator

    dh.addFrame( -pi/2,       0,      20,       0,
               encoder, nullptr, nullptr, nullptr,
          angle_factor,       1,       1,       1);   // Semigorgan

    cout << dh.getH_Matrix(2);      // Prints matrix that describes the position and orientation of the Semigorgan
    return 0;
}