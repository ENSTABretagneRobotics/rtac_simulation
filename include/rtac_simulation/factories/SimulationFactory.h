#ifndef _DEF_RTAC_SIMULATION_SIMULATION_FACTORY_H_
#define _DEF_RTAC_SIMULATION_SIMULATION_FACTORY_H_

#include <yaml-cpp/yaml.h>

#include <rtac_simulation/Simulation.h>

namespace rtac { namespace simulation {

struct SimulationFactory
{
    static Simulation1::Ptr Make1(const std::string& configPath);
    static Simulation1::Ptr Make1(const YAML::Node& config);
};

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_SIMULATION_FACTORY_H_
