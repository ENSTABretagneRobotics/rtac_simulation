#ifndef _DEF_RTAC_SIMULATION_SIMULATION_H_
#define _DEF_RTAC_SIMULATION_SIMULATION_H_

#include <memory>

#include <rtac_simulation/Emitter.h>
#include <rtac_simulation/SensorInstance.h>

namespace rtac { namespace simulation {

class Simulation1
{
    public:

    using Ptr      = std::shared_ptr<Simulation1>;
    using ConstPtr = std::shared_ptr<const Simulation1>;

    protected:

    Simulation1(const Emitter::Ptr&, const SensorInstance2D::Ptr&) {}

    public:
    
    virtual const EmitterBase&      emitter() const = 0;
    virtual       EmitterBase&      emitter() = 0;
    virtual const SensorInstance2D& receiver() const = 0;
    virtual       SensorInstance2D& receiver() = 0;
};

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_SIMULATION_H_

