#ifndef _DEF_RTAC_SIMULATION_EMITTER_H_
#define _DEF_RTAC_SIMULATION_EMITTER_H_

#include <iostream>
#include <memory>

#include <rtac_base/types/Pose.h>
#include <rtac_base/cuda/DeviceVector.h>
#include <rtac_base/cuda/geometry.h>

#include <rtac_simulation/Antenna.h>

namespace rtac { namespace simulation {

struct EmitterView
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
        return this->pose.orientation()*this->initialDirections[idx];
    }
    __device__ Complex<float> sample_value(uint32_t idx) {
        return initialValues[idx];
    }
    #endif //RTAC_CUDACC
};

class Emitter : public Antenna
{
    public:

    using Ptr      = std::shared_ptr<Emitter>;
    using ConstPtr = std::shared_ptr<const Emitter>;

    using Pose      = rtac::Pose<float>;
    using DataShape = Antenna::DataShape;

    protected:

    cuda::DeviceVector<float3>         directions_;
    cuda::DeviceVector<Complex<float>> initialValues_;

    Emitter(const cuda::DeviceVector<float3>& rayDirs,
            Directivity::ConstPtr directivity,
            const Pose& pose = Pose());

    void load_initial_values();
    static cuda::DeviceVector<float3>
        generate_polar_directions(float resolution,
                                  float bearingAperture,
                                  float elevationAperture);

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

    EmitterView view() const {
        EmitterView res;
        res.pose              = this->pose_;
        res.size              = this->size();
        res.initialDirections = this->directions_.data();
        res.initialValues     = this->initialValues_.data();
        return res;
    }
};


} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_EMITTER_H_
