#include <rtac_simulation/geometry.h>
using namespace rtac::simulation;

float get_index(const Matrix3<float>& mat)
{
    return mat(1,2);
}

float get_index_raw(const std::array<float,9>& mat) {
    return mat[7];
}

const Matrix3<float> product(const Matrix3<float>& lhs,
                             const Matrix3<float>& rhs)
{
    Matrix3<float> res;
    res(0,0) = lhs(0,0)*rhs(0,0) + lhs(0,1)*rhs(1,0) + lhs(0,2)*rhs(2,0);
    res(0,1) = lhs(0,0)*rhs(0,1) + lhs(0,1)*rhs(1,1) + lhs(0,2)*rhs(2,1);
    res(0,2) = lhs(0,0)*rhs(0,2) + lhs(0,1)*rhs(1,2) + lhs(0,2)*rhs(2,2);

    res(1,0) = lhs(1,0)*rhs(0,0) + lhs(1,1)*rhs(1,0) + lhs(1,2)*rhs(2,0);
    res(1,1) = lhs(1,0)*rhs(0,1) + lhs(1,1)*rhs(1,1) + lhs(1,2)*rhs(2,1);
    res(1,2) = lhs(1,0)*rhs(0,2) + lhs(1,1)*rhs(1,2) + lhs(1,2)*rhs(2,2);

    res(2,0) = lhs(2,0)*rhs(0,0) + lhs(2,1)*rhs(1,0) + lhs(2,2)*rhs(2,0);
    res(2,1) = lhs(2,0)*rhs(0,1) + lhs(2,1)*rhs(1,1) + lhs(2,2)*rhs(2,1);
    res(2,2) = lhs(2,0)*rhs(0,2) + lhs(2,1)*rhs(1,2) + lhs(2,2)*rhs(2,2);
    return res;
}

const std::array<float,9> product(const std::array<float,9>& lhs,
                                  const std::array<float,9>& rhs)
{
    std::array<float,9> res;

    res[0] = lhs[0]*rhs[0] + lhs[3]*rhs[1] + lhs[6]*rhs[2];
    res[3] = lhs[0]*rhs[3] + lhs[3]*rhs[4] + lhs[6]*rhs[5];
    res[6] = lhs[0]*rhs[6] + lhs[3]*rhs[7] + lhs[6]*rhs[8];

    res[1] = lhs[1]*rhs[0] + lhs[4]*rhs[1] + lhs[7]*rhs[2];
    res[4] = lhs[1]*rhs[3] + lhs[4]*rhs[4] + lhs[7]*rhs[5];
    res[7] = lhs[1]*rhs[6] + lhs[4]*rhs[7] + lhs[7]*rhs[8];

    res[2] = lhs[2]*rhs[0] + lhs[5]*rhs[1] + lhs[8]*rhs[2];
    res[5] = lhs[2]*rhs[3] + lhs[5]*rhs[4] + lhs[8]*rhs[5];
    res[8] = lhs[2]*rhs[6] + lhs[5]*rhs[7] + lhs[8]*rhs[8];

    return res;
}



