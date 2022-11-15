#ifndef _DEF_RTAC_SIMULATION_POSE_3D_H_
#define _DEF_RTAC_SIMULATION_POSE_3D_H_

#include <rtac_base/cuda/geometry.h>

namespace rtac { namespace simulation {

template <typename T> using Matrix3    = rtac::cuda::Matrix3<T>;
template <typename T> using DevicePose = rtac::cuda::DevicePose<T>;

} //namespace simulation
} //namespace rtac
#endif //_DEF_RTAC_SIMULATION_POSE_3D_H_
