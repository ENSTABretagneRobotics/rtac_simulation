#ifndef _DEF_RTAC_SIMULATION_RECEIVER_H_
#define _DEF_RTAC_SIMULATION_RECEIVER_H_

#include <iostream>
#include <memory>

#include <rtac_simulation/common.h>
#include <rtac_simulation/Antenna.h>

namespace rtac { namespace simulation {

template <typename T, typename SampleT>
struct ReceiverView
{
    using SampleType = SampleT;

    DevicePose<float>  pose;
    DirectivityView<T> directivity;
    std::size_t        size;
    SampleType*        samples;

    #ifdef RTAC_CUDACC

    template <typename T2>
    __device__ void set_sample(std::size_t idx, const Sample<T2,float3>& sample)
    {
        // sample position is expected to be expressed in the world frame
        float3 p     = this->pose.to_local_frame(sample.position);
        //samples[idx] = SampleT::Make(sample.datum*directivity(-normalized(p)), p);
        samples[idx] = SampleT::Make(sample.datum, p);
    }

    #endif //RTAC_CUDACC
};

template <typename T, typename SampleT>
class Receiver : public Antenna<T>
{
    public:

    using Ptr      = std::shared_ptr<Receiver<T,SampleT>>;
    using ConstPtr = std::shared_ptr<const Receiver<T,SampleT>>;

    using DataShape = typename Antenna<T>::DataShape;

    using SampleType = SampleT;

    protected:

    DeviceVector<SampleType> receivedSamples_;

    Receiver(std::size_t sampleCount,
             typename Directivity<T>::Ptr directivity, 
             const Pose& pose = Pose());

    public:

    static Ptr Create(std::size_t sampleCount,
                      typename Directivity<T>::Ptr directivity, 
                      const Pose& pose = Pose());

    DeviceVector<SampleType>&       samples()       { return receivedSamples_; }
    const DeviceVector<SampleType>& samples() const { return receivedSamples_; }

    ReceiverView<T,SampleType> view() {
        ReceiverView<T,SampleType> res;

        res.pose        = this->pose_;
        res.directivity = this->directivity()->view();
        res.size        = this->receivedSamples_.size();
        res.samples     = this->receivedSamples_.data();

        return res;
    }
};

template <typename T, typename S>
Receiver<T,S>::Receiver(std::size_t sampleCount,
                        typename Directivity<T>::Ptr directivity, 
                        const Pose& pose) :
    Antenna<T>(directivity, pose),
    receivedSamples_(sampleCount)
{}

template <typename T, typename S>
typename Receiver<T,S>::Ptr Receiver<T,S>::Create(
           std::size_t sampleCount,
           typename Directivity<T>::Ptr directivity, 
           const Pose& pose)
{
    return Ptr(new Receiver<T,S>(sampleCount, directivity, pose));
}

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_RECEIVER_H_

