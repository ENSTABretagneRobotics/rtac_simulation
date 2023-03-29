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

template <typename T> struct ReceiverView;

struct ReceiverViewBase
{
    using Pose = rtac::Pose<float>;

    Pose            pose;
    DirectivityView directivity;
    std::size_t     size;
    void*           samples;
    
    template <typename T>
    RTAC_HOSTDEVICE ReceiverView<T>& cast() {
        return *static_cast<ReceiverView<T>*>(this);
    }
    template <typename T>
    RTAC_HOSTDEVICE const ReceiverView<T>& cast() const {
        return *static_cast<const ReceiverView<T>*>(this);
    }
};

template <typename T>
struct ReceiverView : public ReceiverViewBase
{
    using SampleType = T;

    RTAC_HOSTDEVICE SampleType& sample(unsigned int idx) {
        return static_cast<SampleType*>(samples)[idx];
    }
    RTAC_HOSTDEVICE const SampleType& sample(unsigned int idx) const {
        return static_cast<const SampleType*>(samples)[idx];
    }

    #ifdef RTAC_CUDACC

    __device__ void set_sample(std::size_t idx,
                               const Complex<float>& value,
                               float travel,
                               const float3& direction)
    {
        float3 localDir = this->pose.rotation_matrix().transpose()*(-direction);
        //samples[idx] = SampleType::Make(directivity(localDir)*value,
        //                                travel, localDir);
        this->sample(idx) = SampleType::Make(value, travel, localDir);
    }

    __device__ void set_null_sample(std::size_t idx)
    {
        this->sample(idx) = SampleType::Null();
    }

    #endif //RTAC_CUDACC
};

void sort(rtac::cuda::DeviceVector<SimSample1D>& samples);
void sort(rtac::cuda::DeviceVector<SimSample2D>& samples);
void sort(rtac::cuda::DeviceVector<SimSample3D>& samples);

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_RECEIVER_H_

