#ifndef _DEF_RTAC_SIMULATION_RECEIVER_H_
#define _DEF_RTAC_SIMULATION_RECEIVER_H_

#include <iostream>
#include <memory>

#include <rtac_base/types/Pose.h>
#include <rtac_base/cuda/geometry.h>
#include <rtac_base/cuda/DeviceVector.h>

#include <rtac_simulation/Sample.h>
#include <rtac_simulation/Directivity.h>

namespace rtac { namespace simulation {

template <typename T>
struct ReceiverView
{
    using SampleType = T;
    using Pose = rtac::Pose<float>;


    Pose            pose;
    DirectivityView directivity;
    std::size_t     size;
    SampleType*     samples;

    #ifdef RTAC_CUDACC

    __device__ void set_sample(std::size_t idx,
                               const Complex<float>& value,
                               float travel,
                               const float3& direction)
    {
        float3 localDir = this->pose.rotation_matrix().transpose()*(-direction);
        //samples[idx] = SampleType::Make(directivity(localDir)*value,
        //                                travel, localDir);
        samples[idx] = SampleType::Make(value, travel, localDir);
    }

    __device__ void set_null_sample(std::size_t idx)
    {
        samples[idx] = SampleType::Null();
    }

    #endif //RTAC_CUDACC
};

void sort(rtac::cuda::DeviceVector<SimSample1D>& samples);
void sort(rtac::cuda::DeviceVector<SimSample2D>& samples);
void sort(rtac::cuda::DeviceVector<SimSample3D>& samples);

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_RECEIVER_H_

