#include <rtac_simulation/OptixSimulation.h>

namespace rtac { namespace simulation {

OptixSimulation1::OptixSimulation1(const Emitter::Ptr& emitter,
                                   const SensorInstance::Ptr& receiver) :
    Simulation1(emitter, receiver),
    emitter_(emitter),
    receiver_(receiver),
    rayCaster_(RayCaster::Create())
{}

void OptixSimulation1::run()
{
    if(auto receiverPtr = std::dynamic_pointer_cast<SensorInstance2D>(receiver_)) {
        rayCaster_->trace(*emitter_, *receiverPtr, hitPoints_);
    }
    else {
        throw std::runtime_error("SensorInstance1D not implemented yet");
    }
}

} //namespace simulation
} //namespace rtac
