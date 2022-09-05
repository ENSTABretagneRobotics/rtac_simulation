#include <rtac_simulation/geometry.h>
using namespace rtac::simulation;

__global__ float get_index(const Matrix3<float>& mat)
{
    return mat(1,2);
}

__global__ float get_index_raw(const std::array<float,9>& mat) {
    return mat[7];
}

__global__ const Matrix3<float> product(const Matrix3<float>& lhs,
                                        const Matrix3<float>& rhs)
{
    return lhs*rhs;
}

__global__ const std::array<float,9> product(const std::array<float,9>& lhs,
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



