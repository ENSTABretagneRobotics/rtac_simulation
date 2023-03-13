#ifndef _DEF_RTAC_SIMULATION_FACTORIES_EMITTER_FACTORIES_H_
#define _DEF_RTAC_SIMULATION_FACTORIES_EMITTER_FACTORIES_H_

#include <vector>

#include <yaml-cpp/yaml.h>

#include <rtac_simulation/Emitter.h>
#include <rtac_simulation/factories/utilities.h>

namespace rtac { namespace simulation {

struct EmitterFactory
{
    static EmitterBase::Ptr Make(const YAML::Node& config);
    static EmitterBase::Ptr Make(const std::string& configPath);
};

} //namespace simulation
} //namespace rtac


#endif //_DEF_RTAC_SIMULATION_FACTORIES_EMITTER_FACTORIES_H_
