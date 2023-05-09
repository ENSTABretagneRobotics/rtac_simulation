#include <rtac_simulation/OptixSimulation.h>

#include <yaml-cpp/yaml.h>

#include <rtac_simulation/factories/utilities.h>
#include <rtac_simulation/factories/EmitterFactory.h>
#include <rtac_simulation/factories/SensorInfoFactory.h>

namespace rtac { namespace simulation {

OptixSimulation1::OptixSimulation1(const Emitter::Ptr& emitter,
                                   const SensorInstance::Ptr& receiver) :
    Simulation1(emitter, receiver),
    emitter_(emitter),
    receiver_(receiver),
    rayCaster_(RayCaster::Create(emitter, receiver))
{}

OptixSimulation1::Ptr OptixSimulation1::Create(const Emitter::Ptr& emitter,
                                               const SensorInstance::Ptr& receiver)
{
    return Ptr(new OptixSimulation1(emitter, receiver));
}

OptixSimulation1::Ptr OptixSimulation1::Create(const EmitterBase::Ptr& emitter,
                                               const SensorInstance::Ptr& receiver)
{
    auto emitterPtr = std::dynamic_pointer_cast<Emitter>(emitter);
    if(!emitterPtr) {
        throw std::runtime_error("Invalid EmitterBase child type. Should be Emitter::Ptr.");
    }
    return Ptr(new OptixSimulation1(emitterPtr, receiver));
}

OptixSimulation1::Ptr OptixSimulation1::Create(const std::string& emitterFilename,
                                               const std::string& receiverFilename)
{
    auto emitterPath  = FileFinder::FindOne(emitterFilename);
    if(emitterPath == FileFinder::NotFound) {
        throw ConfigError() << " : could not find config file '" << emitterFilename << "'";
    }
    auto receiverPath = FileFinder::FindOne(receiverFilename);
    if(receiverPath == FileFinder::NotFound) {
        throw ConfigError() << " : could not find config file '" << receiverFilename << "'";
    }

    return Create(EmitterFactory::Make(emitterPath),
                  SensorFactory::Make(receiverPath));
}

optix::ObjectInstance::Ptr OptixSimulation1::add_object(const cuda::DeviceMesh<>::Ptr& mesh)
{
    return rayCaster_->add_object(mesh);
}

bool OptixSimulation1::run()
{
    //if(auto receiverPtr = std::dynamic_pointer_cast<SensorInstance2D>(receiver_)) {
    //    rayCaster_->trace(*emitter_, *receiverPtr, hitPoints_);
    //}
    //else {
    //    throw std::runtime_error("SensorInstance1D not implemented yet");
    //}

    if(!this->emitterPoses_) {
        throw std::runtime_error("Receiver pose source was not set");
    }
    if(this->emitterPoses_->remaining() != 0) {
        emitter_->pose() = this->emitterPoses_->next_pose();
    }
    else {
        return false;
    }
    if(!this->receiverPoses_) {
        throw std::runtime_error("Receiver pose source was not set");
    }
    if(this->receiverPoses_->remaining() != 0) {
        receiver_->pose() = this->receiverPoses_->next_pose();
    }
    else {
        return false;
    }

    rayCaster_->trace(hitPoints_);
    receiver_->compute_output();

    for(const auto& sink : this->sinks_) {
        sink->set_output(receiver_);
    }

    return true;
}

} //namespace simulation
} //namespace rtac
