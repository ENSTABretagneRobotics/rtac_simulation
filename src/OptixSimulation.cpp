#include <rtac_simulation/OptixSimulation.h>

namespace rtac { namespace simulation {

OptixSimulation1::OptixSimulation1(const Emitter::Ptr& emitter,
                                   const SensorInstance2D::Ptr& receiver) :
    Simulation1(emitter, receiver),
    emitter_(emitter),
    receiver_(receiver),
    rayCaster_(RayCaster::Create())
{}


} //namespace simulation
} //namespace rtac
