#ifndef _DEF_RTAC_SIMULATION_OPTIX_SIMULATION_H_
#define _DEF_RTAC_SIMULATION_OPTIX_SIMULATION_H_

#include <memory>

#include <rtac_simulation/Simulation.h>
#include <rtac_simulation/RayCaster.h>


namespace rtac { namespace simulation {

class OptixSimulation1 : public Simulation1
{
    public:

    using Ptr      = std::shared_ptr<OptixSimulation1>;
    using ConstPtr = std::shared_ptr<const OptixSimulation1>;

    protected:

    Emitter::Ptr          emitter_;
    SensorInstance2D::Ptr receiver_;

    RayCaster::Ptr rayCaster_;

    OptixSimulation1(const Emitter::Ptr& emitter,
                     const SensorInstance2D::Ptr& receiver);

    public:

    static Ptr Create(const Emitter::Ptr& emitter,
                      const SensorInstance2D::Ptr& receiver)
    {
        return Ptr(new OptixSimulation1(emitter, receiver));
    }
    
    virtual const EmitterBase&      emitter()  const { return *emitter_; }
    virtual       EmitterBase&      emitter()        { return *emitter_; }
    virtual const SensorInstance2D& receiver() const { return *receiver_; }
    virtual       SensorInstance2D& receiver()       { return *receiver_; }

    const RayCaster& ray_caster() const { return *rayCaster_; }
          RayCaster& ray_caster()       { return *rayCaster_; }

};

} //namespace simulation
} //namespace rtac


#endif //_DEF_RTAC_SIMULATION_OPTIX_SIMULATION_H_
