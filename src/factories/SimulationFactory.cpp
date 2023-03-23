#include <rtac_simulation/factories/SimulationFactory.h>
#include <rtac_simulation/factories/EmitterFactory.h>
#include <rtac_simulation/factories/SensorInfoFactory.h>

namespace rtac { namespace simulation {

Simulation1::Ptr SimulationFactory::Make1(const std::string& configPath)
{
    FileFinder::Get()->add_search_path(configPath);
    return SimulationFactory::Make1(YAML::LoadFile(configPath));
}

Simulation1::Ptr SimulationFactory::Make1(const YAML::Node& config)
{
    auto emitter  = EmitterFactory::Make(config["emitter"]);
    auto receiver = SensorFactory::Make(config["receiver"]);

    return nullptr;
}

} //namespace simulation
} //namespace rtac

