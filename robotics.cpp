#include "robotics.hpp"

using namespace rrt; // Ryan's Robotics Toolbox

enum { theta = 0, d = 1, a = 2, alpha =3 };

constexpr void D_H_Table::addFrame(double  theta_offset, double  d_offset, double  a_offset, double  alpha_offset,
                                   double* theta_driver, double* d_driver, double* a_driver, double* alpha_driver) noexcept
{
    offsets[theta].emplace_back(theta_offset);
    offsets[    d].emplace_back(d_offset);
    offsets[    a].emplace_back(a_offset);
    offsets[alpha].emplace_back(alpha_offset);
    
    drivers[theta].emplace_back(theta_driver);
    drivers[    d].emplace_back(d_driver);
    drivers[    a].emplace_back(a_driver);
    drivers[alpha].emplace_back(alpha_driver);
}

Matrix<4,4> D_H_Table::getH_Matrix(uint32_t frame, uint32_t ref_frame = 0) noexcept
{
    if(frame == ref_frame)
        return identity<4>;
    else if(frame > ref_frame)
        return H(offsets[frame][theta] + (drivers[frame][theta]?*drivers[frame][theta]:0),
                 offsets[frame][    d] + (drivers[frame][    d]?*drivers[frame][    d]:0),
                 offsets[frame][    a] + (drivers[frame][    a]?*drivers[frame][    a]:0),
                 offsets[frame][alpha] + (drivers[frame][alpha]?*drivers[frame][alpha]:0)) * getH_Matrix(frame - 1, ref_frame);
    else
        return identity<4>; // Unimplimented, requires inverse matrices, which aren't implimented, at the time of writing this
    
    
}