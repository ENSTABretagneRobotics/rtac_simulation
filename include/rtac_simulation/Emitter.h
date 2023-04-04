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

    float frequency_;
    Pose  pose_;

    EmitterBase(float frequency, const Pose& pose) :
         frequency_(frequency), pose_(pose)
    {}

    public:

    const Pose& pose() const { return pose_; }
          Pose& pose()       { return pose_; }
    float frequency()  const { return frequency_; }
    
    unsigned int size() const { return this->ray_count(); }
    virtual unsigned int ray_count() const = 0;
};

struct EmitterView
{
    using Pose = rtac::Pose<float>;

    Pose                  pose;
    std::size_t           size;
    float                 frequency;
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

    static cuda::DeviceVector<float3>
        generate_polar_directions(float resolution,
                                  float bearingAperture,
                                  float elevationAperture);
    static cuda::DeviceVector<float3>
        generate_icosahedron_directions(float resolution,
                                        float bearingAperture,
                                        float elevationAperture);

    protected:

    Directivity::ConstPtr              directivity_;
    cuda::DeviceVector<float3>         directions_;
    cuda::DeviceVector<Complex<float>> initialValues_;

    Emitter(const cuda::DeviceVector<float3>& rayDirs,
            Directivity::ConstPtr directivity,
            float frequency,
            const Pose& pose = Pose());

    void load_initial_values();

    public:

    static Ptr Create(const cuda::DeviceVector<float3>& rayDirs,
                      Directivity::ConstPtr directivity,
                      float frequency,
                      const Pose& pose = Pose());

    static Ptr Create(float resolution,
                      float bearingAperture,
                      float elevationAperture,
                      Directivity::ConstPtr directivity,
                      float frequency,
                      const Pose& pose = Pose());
    
    const Directivity::ConstPtr& directivity() const { return directivity_; }
    unsigned int ray_count() const { return directions_.size(); }

    EmitterView view() const {
        EmitterView res;
        res.pose              = this->pose();
        res.size              = this->ray_count();
        res.frequency         = this->frequency();
        res.initialDirections = this->directions_.data();
        res.initialValues     = this->initialValues_.data();
        return res;
    }
};


} //namespace simulation
} //namespace rtac

inline std::ostream& operator<<(std::ostream& os, const rtac::simulation::EmitterBase& emitter)
{
    os << "EmitterBase :"
       << "\n- pose : " << emitter.pose()
       << "\n- ray_count : " << emitter.ray_count();
    return os;
}


#endif //_DEF_RTAC_SIMULATION_EMITTER_H_
