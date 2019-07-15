#pragma once

#include <iostream>
#include <memory>
#include <cmath>
#include "Mat.hpp"

namespace rrt // Ryan's Robotics Toolbox
{
    enum class Axis { _X_, _Y_, _Z_ };
    double const pi = acos(-1);

    template<Axis axis>
    inline Matrix<4,4> Rot(double theta) noexcept
    {
        if constexpr(axis == Axis::_X_)
            return Matrix<4,4>({
                { 1,           0,            0,     0 }, // Note: You are looking at the transpose of the matrix
                { 0,  cos(theta),   sin(theta),     0 }, //       because the constructor takes a set of vectors
                { 0, -sin(theta),   cos(theta),     0 },
                { 0,           0,            0,     1 }
            });
        else if constexpr (axis == Axis::_Y_)
            return Matrix<4,4>({
                { cos(theta),    0, sin(theta),     0 },
                {          0,    1,          0,     0 },
                {-sin(theta),    0, cos(theta),     0 },
                {          0,    0,          0,     1 }
            }).transpose();
        else if constexpr (axis == Axis::_Z_)
            return Matrix<4,4>({
                { cos(theta),    sin(theta),   0,   0 },
                {-sin(theta),    cos(theta),   0,   0 },
                {          0,             0,   1,   0 },
                {          0,             0,   0,   1 }
            });
        else return identity<4>;
    }

    template<Axis axis>
    inline Matrix<4,4> Trans(double dist) noexcept
    {
        if constexpr(axis == Axis::_X_)
            return Matrix<4,4>({
                {    1,    0,    0,    0 }, // Note: You are looking at the transpose of the matrix
                {    0,    1,    0,    0 }, //       because the constructor takes a set of vectors
                {    0,    0,    1,    0 },
                { dist,    0,    0,    1 }
            });
        else if constexpr (axis == Axis::_Y_)
            return Matrix<4,4>({
                { 1,    0,    0,    0 },
                { 0,    1,    0,    0 },
                { 0,    0,    1,    0 },
                { 0, dist,    0,    1 }
            });
        else if constexpr (axis == Axis::_Z_)
            return Matrix<4,4>({
                { 1,    0,    0,    0 },
                { 0,    1,    0,    0 },
                { 0,    0,    1,    0 },
                { 0, dist,    0,    1 }
            });
            
        else return identity<4>;
    }

    inline Matrix<4,4> H(double theta, double d, double a, double alpha) noexcept
    {
        return Rot<Axis::_Z_>(theta) * Trans<Axis::_Z_>(d) * Trans<Axis::_X_>(a) * Rot<Axis::_X_>(alpha);
    }

    class D_H_Table
    {
        std::vector<double*> drivers[4];
        std::vector<double> offsets[4]; 
        std::vector<double> factors[4]; 

    public:
        D_H_Table();

        /// Input nullptr for unused drivers, and let the D_H_Table handle object lifetime of the drivers
        /// See documentation for what theta, d, a, and alpha refer to.
        /// factors are multipliers to adjust to standard units
        void addFrame(
            double  theta_offset,     double  d_offset,     double  a_offset,     double  alpha_offset,
            double* theta_driver,     double* d_driver,     double* a_driver,     double* alpha_driver,
            double  theta_factor = 1, double  d_factor = 1, double  a_factor = 1, double  alpha_factor = 1) noexcept;

        Matrix<4,4> getH_Matrix(uint32_t frame, uint32_t ref_frame = 0) noexcept;
    };

    class Robot_System
    {

    };
}