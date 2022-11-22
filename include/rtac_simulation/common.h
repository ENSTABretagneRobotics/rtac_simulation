#ifndef _DEF_RTAC_SIMULATION_COMMON_H_
#define _DEF_RTAC_SIMULATION_COMMON_H_

#include <rtac_base/cuda_defines.h>
#include <rtac_base/cuda/utils.h>

#include <rtac_base/types/VectorView.h>
#include <rtac_base/types/Pose.h>
#include <rtac_base/types/Image.h>

#include <rtac_base/cuda/DeviceVector.h>
#include <rtac_base/cuda/PinnedVector.h>
#include <rtac_base/cuda/HostVector.h>

namespace rtac { namespace simulation {

// Som type aliases for convenience
template <typename T> using Complex      = rtac::cuda::Complex<T>;
template <typename T> using VectorView   = rtac::VectorView<T>;
template <typename T> using DeviceVector = rtac::cuda::DeviceVector<T>;
template <typename T> using PinnedVector = rtac::cuda::PinnedVector<T>;
template <typename T> using HostVector   = rtac::cuda::HostVector<T>;
template <typename T> using ImageView    = rtac::ImageView<T>;
template <typename T> using DeviceImage  = rtac::Image<T, DeviceVector>;

template <typename T, template<typename>class ContainerT>
using Image = rtac::Image<T,ContainerT>;

using Pose = rtac::Pose<float>;

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_COMMON_H_
