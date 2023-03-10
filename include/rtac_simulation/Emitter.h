#ifndef _DEF_RTAC_SIMULATION_EMITTER_H_
#define _DEF_RTAC_SIMULATION_EMITTER_H_

#include <iostream>
#include <memory>

#include <rtac_base/types/Pose.h>
#include <rtac_base/cuda/DeviceVector.h>
#include <rtac_base/cuda/geometry.h>

#include <rtac_simulation/Directivity.h>

namespace rtac { namespace simulation {

class EmitterBase
{
    public:

    using Ptr      = std::shared_ptr<EmitterBase>;
    using ConstPtr = std::shared_ptr<const EmitterBase>;

    using Pose = rtac::Pose<float>;

    protected:

    Pose pose_;

    EmitterBase(const Pose& pose) : pose_(pose) {}

    public:

    const Pose& pose() const { return pose_; }
          Pose& pose()       { return pose_; }
    
    unsigned int size() const { return this->ray_count(); }
    virtual unsigned int ray_count() const = 0;
};

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

class Emitter : public EmitterBase
{
    public:

    using Ptr      = std::shared_ptr<Emitter>;
    using ConstPtr = std::shared_ptr<const Emitter>;
    
    using Pose = EmitterBase::Pose;

    protected:

    Directivity::ConstPtr              directivity_;
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
    
    const Directivity::ConstPtr& directivity() const { return directivity_; }
    unsigned int ray_count() const { return directions_.size(); }

    EmitterView view() const {
        EmitterView res;
        res.pose              = this->pose();
        res.size              = this->ray_count();
        res.initialDirections = this->directions_.data();
        res.initialValues     = this->initialValues_.data();
        return res;
    }
};


} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_EMITTER_H_
