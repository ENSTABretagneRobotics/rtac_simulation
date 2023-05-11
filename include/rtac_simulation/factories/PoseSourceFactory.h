#ifndef _DEF_RTAC_SIMULATION_POSE_SOURCE_FACTORY_H_
#define _DEF_RTAC_SIMULATION_POSE_SOURCE_FACTORY_H_

#include <yaml-cpp/yaml.h>

#include <rtac_simulation/PoseSource.h>

namespace rtac { namespace simulation {

struct PoseSourceFactory
{
    using Pose = PoseSource::Pose;

    static PoseSource::Ptr Make(const YAML::Node&  config);
    static PoseSource::Ptr Make(const std::string& configPath);

    static PoseSource::Ptr parse_static(const YAML::Node& config);
};

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_POSE_SOURCE_FACTORY_H_
