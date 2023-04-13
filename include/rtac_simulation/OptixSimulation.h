#ifndef _DEF_RTAC_SIMULATION_OPTIX_SIMULATION_H_
#define _DEF_RTAC_SIMULATION_OPTIX_SIMULATION_H_

#include <memory>


#include <rtac_base/cuda/DeviceMesh.h>
#include <rtac_optix/ObjectInstance.h>

#include <rtac_simulation/Simulation.h>
#include <rtac_simulation/RayCaster.h>

namespace rtac { namespace simulation {

class OptixSimulation1 : public Simulation1
{
    public:

    using Ptr      = std::shared_ptr<OptixSimulation1>;
    using ConstPtr = std::shared_ptr<const OptixSimulation1>;

    protected:

    Emitter::Ptr        emitter_;
    SensorInstance::Ptr receiver_;

    RayCaster::Ptr rayCaster_;
    cuda::CudaVector<float3> hitPoints_;


    OptixSimulation1(const Emitter::Ptr& emitter,
                     const SensorInstance::Ptr& receiver);

    //Emitter::Ptr        emitter_ptr()  { return emitter_;  }
    //SensorInstance::Ptr receiver_ptr() { return receiver_; }

    public:

    static Ptr Create(const Emitter::Ptr& emitter,
                      const SensorInstance::Ptr& receiver);
    static Ptr Create(const EmitterBase::Ptr& emitter,
                      const SensorInstance::Ptr& receiver);
    static Ptr Create(const std::string& emitterFilename,
                      const std::string& receiverFilename);
    
    virtual const EmitterBase&    emitter()  const { return rayCaster_->emitter(); } 
    virtual       EmitterBase&    emitter()        { return rayCaster_->emitter(); } 
    virtual const SensorInstance& receiver() const { return rayCaster_->receiver(); } 
    virtual       SensorInstance& receiver()       { return rayCaster_->receiver(); }

    const RayCaster& ray_caster() const { return *rayCaster_; }
          RayCaster& ray_caster()       { return *rayCaster_; }

    optix::ObjectInstance::Ptr add_object(const cuda::DeviceMesh<>::Ptr& mesh);
    
    void run();

    const cuda::CudaVector<float3>& hit_points() const { return hitPoints_; }
};

} //namespace simulation
} //namespace rtac


#endif //_DEF_RTAC_SIMULATION_OPTIX_SIMULATION_H_
