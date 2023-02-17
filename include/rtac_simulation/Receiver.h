#ifndef _DEF_RTAC_SIMULATION_RECEIVER_H_
#define _DEF_RTAC_SIMULATION_RECEIVER_H_

#include <iostream>
#include <memory>

#include <rtac_base/types/Pose.h>
#include <rtac_base/cuda/geometry.h>
#include <rtac_base/cuda/DeviceVector.h>

#include <rtac_simulation/Antenna.h>
#include <rtac_simulation/Sample.h>

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

    template <typename T2>
    __device__ void set_sample(std::size_t idx, const Sample<T2,float3>& sample)
    {
        // sample position is expected to be expressed in the world frame
        //float3 p     = this->pose.to_local_frame(sample.position);
        float3 p     = this->pose.rotation_matrix().transpose()
                     * (sample.position - this->pose.translation());
        //samples[idx] = SampleT::Make(sample.datum*directivity(-normalized(p)), p);
        samples[idx] = SampleType::Make(sample.datum, p);
    }

    #endif //RTAC_CUDACC
};

template <typename T>
class Receiver : public Antenna
{
    public:

    using Ptr      = std::shared_ptr<Receiver<T>>;
    using ConstPtr = std::shared_ptr<const Receiver<T>>;

    using Pose      = rtac::Pose<float>;
    using DataShape = Antenna::DataShape;

    using SampleType = T;

    protected:

    cuda::DeviceVector<T> receivedSamples_;

    Receiver(typename Directivity::ConstPtr directivity) : Antenna(directivity) {}

    public:

    static Ptr Create(typename Directivity::ConstPtr directivity) {
        return Ptr(new Receiver<T>(directivity));
    }

    cuda::DeviceVector<T>&       samples()       { return receivedSamples_; }
    const cuda::DeviceVector<T>& samples() const { return receivedSamples_; }

    ReceiverView<T> view() {
        ReceiverView<T> res;

        res.pose        = this->pose_;
        res.directivity = this->directivity()->view();
        res.size        = this->receivedSamples_.size();
        res.samples     = this->receivedSamples_.data();

        return res;
    }
};

template <typename T>
struct ReceiverView2
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

template <typename T>
class Receiver2 : public Antenna
{
    public:

    using Ptr      = std::shared_ptr<Receiver2<T>>;
    using ConstPtr = std::shared_ptr<const Receiver2<T>>;

    using Pose      = rtac::Pose<float>;
    using DataShape = Antenna::DataShape;

    using SampleType = T;

    protected:

    cuda::DeviceVector<T> receivedSamples_;

    Receiver2(typename Directivity::ConstPtr directivity) : Antenna(directivity) {}

    public:

    static Ptr Create(typename Directivity::ConstPtr directivity) {
        return Ptr(new Receiver2<T>(directivity));
    }

    cuda::DeviceVector<T>&       samples()       { return receivedSamples_; }
    const cuda::DeviceVector<T>& samples() const { return receivedSamples_; }

    void sort_received() { sort(receivedSamples_); }

    ReceiverView2<T> view() {
        ReceiverView2<T> res;

        res.pose        = this->pose_;
        res.directivity = this->directivity()->view();
        res.size        = this->receivedSamples_.size();
        res.samples     = this->receivedSamples_.data();

        return res;
    }
};

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_RECEIVER_H_

