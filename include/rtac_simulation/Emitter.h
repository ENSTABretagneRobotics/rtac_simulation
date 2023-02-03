#ifndef _DEF_RTAC_SIMULATION_EMITTER_H_
#define _DEF_RTAC_SIMULATION_EMITTER_H_

#include <iostream>
#include <memory>

#include <rtac_base/types/Pose.h>
#include <rtac_base/cuda/DeviceVector.h>
#include <rtac_base/cuda/geometry.h>

#include <rtac_simulation/Antenna.h>

namespace rtac { namespace simulation {

template <typename T>
struct EmitterView
{
    using Pose = rtac::Pose<float>;

    Pose              pose;
    std::size_t       size;
    const float3*     initialDirections;
    const Complex<T>* initialSampleValues;
    DirectivityView   directivity;

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
        return this->pose.rotation_matrix() * dir;
    }
    #endif //RTAC_CUDACC
};

struct EmitterView2
{
    using Pose = rtac::Pose<float>;

    Pose                  pose;
    std::size_t           size;
    const float3*         initialDirections;
    const Complex<float>* initialValues;

    #ifdef RTAC_CUDACC
    __device__ float3 ray_origin(uint32_t idx) const { 
        // For now origin is center of sensor
        return float3{pose.x(), pose.y(), pose.z()};
    }
    __device__ float3 ray_direction(uint32_t idx) const {
        return this->pose*this->initialDirections[idx];
    }
    __device__ Complex<float> sample_value(uint32_t idx) {
        return initialValues[idx];
    }
    #endif //RTAC_CUDACC
};

class Emitter2 : public Antenna
{
    public:

    using Ptr      = std::shared_ptr<Emitter2>;
    using ConstPtr = std::shared_ptr<const Emitter2>;

    using Pose      = rtac::Pose<float>;
    using DataShape = Antenna::DataShape;

    protected:

    cuda::DeviceVector<float3>         directions_;
    cuda::DeviceVector<Complex<float>> initialValues_;

    Emitter2(const cuda::DeviceVector<float3>& rayDirs,
             Directivity::ConstPtr directivity,
             const Pose& pose = Pose());

    void load_initial_values();

    public:

    static Ptr Create(const cuda::DeviceVector<float3>& rayDirs,
                      Directivity::ConstPtr directivity,
                      const Pose& pose = Pose());

    static Ptr Create(float resolution,
                      float bearingAperture,
                      float elevationAperture,
                      Directivity::ConstPtr directivity,
                      const Pose& pose = Pose());

    std::size_t size() const { return directions_.size(); }

    EmitterView2 view() const {
        EmitterView2 res;
        res.pose              = this->pose_;
        res.size              = this->size();
        res.initialDirections = this->directions_.data();
        res.initialValues     = this->initialValues_.data();
        return res;
    }
};

template <typename T>
class Emitter : public Antenna
{
    public:

    using Ptr      = std::shared_ptr<Emitter<T>>;
    using ConstPtr = std::shared_ptr<const Emitter<T>>;

    using Pose      = rtac::Pose<float>;
    using DataShape = Antenna::DataShape;

    protected:
    
    // remove these
    cuda::DeviceVector<float3>     initialDirections_;
    cuda::DeviceVector<Complex<T>> initialSampleValues_;

    Emitter(const cuda::DeviceVector<float3>& initialDirections,
            typename Directivity::ConstPtr directivity, 
            const Pose& pose = Pose());

    public:

    static Ptr Create(const cuda::DeviceVector<float3>& initialDirections,
                      typename Directivity::ConstPtr directivity, 
                      const Pose& pose = Pose());

    static auto generate_polar_directions(float resolution,
                                          float bearingAperture,
                                          float elevationAperture);

    const cuda::DeviceVector<float3>& initial_directions() const { 
        return initialDirections_;
    }
    const cuda::DeviceVector<Complex<T>>& initial_sample_values() const {
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
Emitter<T>::Emitter(const cuda::DeviceVector<float3>& initialDirections,
                    typename Directivity::ConstPtr directivity, 
                    const Pose& pose) :
    Antenna(directivity, pose),
    initialDirections_(initialDirections),
    initialSampleValues_(initialDirections.size())
{}

template <typename T>
typename Emitter<T>::Ptr Emitter<T>::Create(
           const cuda::DeviceVector<float3>& initialDirections,
           typename Directivity::ConstPtr directivity, 
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

    unsigned int idx = 0;
    unsigned int H   = (int)(elevationAperture / resolution) + 1;
    
    std::vector<float3> data;
    for(int h = 0; h < H; h++) {
        float phi = elevationAperture * (((float)h) / (H - 1) - 0.5f);
        int W = (int)(abs(std::cos(phi)) * bearingAperture   / resolution) + 1;
        for(int w = 0; w < W; w++) {
            float theta = bearingAperture * (((float)w) / (W - 1) - 0.5f);
            data.push_back(float3{std::cos(theta)*std::cos(phi),
                                  std::sin(theta)*std::cos(phi),
                                  std::sin(phi)});
        }
    }
    return cuda::DeviceVector<float3>(data);

    //unsigned int W = (int)(bearingAperture   / resolution) + 1;
    //unsigned int H = (int)(elevationAperture / resolution) + 1;

    //rtac::Image<float3> data({W,H});
    //for(int h = 0; h < H; h++) {
    //    float phi = elevationAperture * (((float)h) / (H - 1) - 0.5f);
    //    for(int w = 0; w < W; w++) {
    //        float theta = bearingAperture * (((float)w) / (W - 1) - 0.5f);
    //        data(h,w) = float3{cos(theta)*cos(phi),
    //                           sin(theta)*cos(phi),
    //                           sin(phi)};
    //    }
    //}

    //return rtac::Image<float3, rtac::cuda::DeviceVector>(data);
}

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_EMITTER_H_
