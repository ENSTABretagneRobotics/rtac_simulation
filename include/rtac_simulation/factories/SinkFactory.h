#ifndef _DEF_RTAC_SIMULATION_FACTORIES_SINK_FACTORY_H_
#define _DEF_RTAC_SIMULATION_FACTORIES_SINK_FACTORY_H_

#include <yaml-cpp/yaml.h>

#include <rtac_simulation/Sink.h>

namespace rtac { namespace simulation {

struct SinkFactory
{
    static Sink::Ptr Make(const YAML::Node&  config);
    static Sink::Ptr Make(const std::string& configPath);
};

} //namespace simulation
} //namespace rtac


#endif //_DEF_RTAC_SIMULATION_FACTORIES_SINK_FACTORY_H_
