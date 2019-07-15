#include "robotics.hpp"

using namespace rrt; // Ryan's Robotics Toolbox

enum { theta = 0, d = 1, a = 2, alpha =3 };

D_H_Table::D_H_Table()
{
    addFrame(0,0,0,0,nullptr,nullptr,nullptr,nullptr);
}

void D_H_Table::addFrame(
    double  theta_offset, double  d_offset, double  a_offset, double  alpha_offset,
    double* theta_driver, double* d_driver, double* a_driver, double* alpha_driver,
    double  theta_factor, double  d_factor, double  a_factor, double  alpha_factor) noexcept
{
    offsets[theta].emplace_back(theta_offset);
    offsets[    d].emplace_back(d_offset);
    offsets[    a].emplace_back(a_offset);
    offsets[alpha].emplace_back(alpha_offset);
    
    drivers[theta].emplace_back(theta_driver);
    drivers[    d].emplace_back(d_driver);
    drivers[    a].emplace_back(a_driver);
    drivers[alpha].emplace_back(alpha_driver);

    factors[theta].emplace_back(theta_factor);
    factors[    d].emplace_back(    d_factor);
    factors[    a].emplace_back(    a_factor);
    factors[alpha].emplace_back(alpha_factor);
}

Matrix<4,4> D_H_Table::getH_Matrix(uint32_t frame, uint32_t ref_frame) noexcept
{
    if(frame == ref_frame)
        return identity<4>;
    else if(frame > ref_frame)
        return H(offsets[theta][frame] + (drivers[theta][frame]?*drivers[theta][frame]:0) * factors[theta][frame],
                 offsets[    d][frame] + (drivers[    d][frame]?*drivers[    d][frame]:0) * factors[    d][frame],
                 offsets[    a][frame] + (drivers[    a][frame]?*drivers[    a][frame]:0) * factors[    a][frame],
                 offsets[alpha][frame] + (drivers[alpha][frame]?*drivers[alpha][frame]:0) * factors[alpha][frame])
                    * getH_Matrix(frame - 1, ref_frame);
    else
        return inverse(getH_Matrix(ref_frame, frame));
}