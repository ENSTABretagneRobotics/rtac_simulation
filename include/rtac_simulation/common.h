#ifndef _DEF_RTAC_SIMULATION_COMMON_H_
#define _DEF_RTAC_SIMULATION_COMMON_H_

#include <rtac_base/cuda_defines.h>
#include <rtac_base/cuda/utils.h>

#include <rtac_base/types/VectorView.h>
#include <rtac_base/types/Pose.h>

#include <rtac_base/cuda/DeviceVector.h>
#include <rtac_base/cuda/PinnedVector.h>
#include <rtac_base/cuda/HostVector.h>

namespace rtac { namespace simulation {

// Som type aliases for convenience
template <typename T> using Complex      = rtac::cuda::Complex<T>;
template <typename T> using VectorView   = rtac::types::VectorView<T>;
template <typename T> using DeviceVector = rtac::cuda::DeviceVector<T>;
template <typename T> using PinnedVector = rtac::cuda::PinnedVector<T>;
template <typename T> using HostVector   = rtac::cuda::HostVector<T>;

using Pose = rtac::types::Pose<float>;

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_COMMON_H_
