#ifndef _DEF_RTAC_SIMULATION_RECEIVER_H_
#define _DEF_RTAC_SIMULATION_RECEIVER_H_

#include <iostream>
#include <memory>

#include <rtac_simulation/common.h>
#include <rtac_simulation/Antenna.h>

namespace rtac { namespace simulation {

template <typename T>
struct ReceiverView
{
    using SampleType = T;

    DevicePose<float> pose;
    DirectivityView   directivity;
    std::size_t       size;
    SampleType*       samples;

    #ifdef RTAC_CUDACC

    template <typename T2>
    __device__ void set_sample(std::size_t idx, const Sample<T2,float3>& sample)
    {
        // sample position is expected to be expressed in the world frame
        float3 p     = this->pose.to_local_frame(sample.position);
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

    using DataShape = Antenna::DataShape;

    using SampleType = T;

    protected:

    DeviceVector<T> receivedSamples_;

    Receiver(std::size_t sampleCount,
             typename Directivity::ConstPtr directivity, 
             const Pose& pose = Pose());

    public:

    static Ptr Create(std::size_t sampleCount,
                      typename Directivity::ConstPtr directivity, 
                      const Pose& pose = Pose());

    DeviceVector<T>&       samples()       { return receivedSamples_; }
    const DeviceVector<T>& samples() const { return receivedSamples_; }

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
Receiver<T>::Receiver(std::size_t sampleCount,
                      typename Directivity::ConstPtr directivity, 
                      const Pose& pose) :
    Antenna(directivity, pose),
    receivedSamples_(sampleCount)
{}

template <typename T>
typename Receiver<T>::Ptr Receiver<T>::Create(
           std::size_t sampleCount,
           typename Directivity::ConstPtr directivity, 
           const Pose& pose)
{
    return Ptr(new Receiver<T>(sampleCount, directivity, pose));
}

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_RECEIVER_H_

