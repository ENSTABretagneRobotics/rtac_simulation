#ifndef _DEF_RTAC_SIMULATION_EMITTER_H_
#define _DEF_RTAC_SIMULATION_EMITTER_H_

#include <iostream>
#include <memory>

#include <rtac_simulation/common.h>
#include <rtac_simulation/Antenna.h>

namespace rtac { namespace simulation {

template <typename T>
struct EmitterView
{
    DevicePose<float> pose;
    std::size_t       size;
    const float3*     initialDirections;
    const Complex<T>* initialSampleValues;
    DirectivityView<T> directivity;

    #ifdef RTAC_CUDACC
    __device__ float3 ray_origin(uint32_t idx) const { 
        // For now origin is center of sensor
        return this->pose.translation();
    }
    __device__ float3 ray_direction(uint32_t idx) const {
        return this->pose*this->initialDirections[idx];
    }
    //__device__ Complex<T> sample_value(uint32_t idx) const {
    //    return initialSampleValues[idx];
    //}
    __device__ Complex<T> sample_value(const float3& direction) {
        return directivity(direction);
    }
    __device__ float3 ray_direction(const float3& dir) const {
        return this->pose.R_ * dir;
    }
    #endif //RTAC_CUDACC
};

template <typename T>
class Emitter : public Antenna<T>
{
    public:

    using Ptr      = std::shared_ptr<Emitter<T>>;
    using ConstPtr = std::shared_ptr<const Emitter<T>>;

    using DataShape = typename Antenna<T>::DataShape;

    protected:
    
    // remove these
    DeviceVector<float3>     initialDirections_;
    DeviceVector<Complex<T>> initialSampleValues_;

    Emitter(const DeviceVector<float3>& initialDirections,
            typename Directivity<T>::Ptr directivity, 
            const Pose& pose = Pose());

    public:

    static Ptr Create(const DeviceVector<float3>& initialDirections,
                      typename Directivity<T>::Ptr directivity, 
                      const Pose& pose = Pose());

    static auto generate_polar_directions(float resolution,
                                          float bearingAperture,
                                          float elevationAperture);

    const DeviceVector<float3>& initial_directions() const { 
        return initialDirections_;
    }
    const DeviceVector<Complex<T>>& initial_sample_values() const {
        return initialSampleValues_;
    }

    EmitterView<T> view() const {
        EmitterView<T> res;
        res.pose                = this->pose_;
        res.size                = this->initialDirections_.size();
        res.initialDirections   = this->initialDirections_.data();
        res.initialSampleValues = this->initialSampleValues_.data();
        res.directivity         = this->directivity()->view();
        return res;
    }
};

template <typename T>
Emitter<T>::Emitter(const DeviceVector<float3>& initialDirections,
                    typename Directivity<T>::Ptr directivity, 
                    const Pose& pose) :
    Antenna<T>(directivity, pose),
    initialDirections_(initialDirections),
    initialSampleValues_(initialDirections.size())
{}

template <typename T>
typename Emitter<T>::Ptr Emitter<T>::Create(
           const DeviceVector<float3>& initialDirections,
           typename Directivity<T>::Ptr directivity, 
           const Pose& pose)
{
    return Ptr(new Emitter<T>(initialDirections, directivity, pose));
}


template <typename T>
auto Emitter<T>::generate_polar_directions(float resolution,
                                           float bearingAperture,
                                           float elevationAperture)
{
    resolution        *= M_PI  / 180.0f;
    bearingAperture   *= M_PI  / 180.0f;
    elevationAperture *= M_PI  / 180.0f;
    unsigned int W = (int)(bearingAperture   / resolution) + 1;
    unsigned int H = (int)(elevationAperture / resolution) + 1;

    rtac::types::Image<float3, rtac::cuda::HostVector> data({W,H});
    for(int h = 0; h < H; h++) {
        float phi = elevationAperture * (((float)h) / (H - 1) - 0.5f);
        for(int w = 0; w < W; w++) {
            float theta = bearingAperture * (((float)w) / (W - 1) - 0.5f);
            data(h,w) = float3{cos(theta)*cos(phi),
                               sin(theta)*cos(phi),
                               sin(phi)};
        }
    }

    return rtac::types::Image<float3, rtac::cuda::DeviceVector>(data);
}

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_EMITTER_H_
