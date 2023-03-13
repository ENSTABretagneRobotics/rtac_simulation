#ifndef _DEF_RTAC_SIMULATION_SENSOR_INFO_FACTORY_H_
#define _DEF_RTAC_SIMULATION_SENSOR_INFO_FACTORY_H_

#include <vector>

#include <yaml-cpp/yaml.h>

#include <rtac_base/types/Linspace.h>

#include <rtac_simulation/Directivity.h>
#include <rtac_simulation/Waveform.h>
#include <rtac_simulation/BeamDirectivity.h>
#include <rtac_simulation/SensorInfo.h>
#include <rtac_simulation/SensorInstance.h>
#include <rtac_simulation/factories/utilities.h>

namespace rtac { namespace simulation {

struct SensorFactory
{
    static SensorInstance::Ptr Make(const YAML::Node& config);
    static SensorInstance::Ptr Make(const std::string& configPath);
};

struct SensorInfoFactory2D
{
    static SensorInfo::Ptr Make(const YAML::Node& config);
    static SensorInfo::Ptr Make(const std::string& configPath);

    static SensorInfo2D::Ptr Make_FrontScanInfo(const YAML::Node& config);

    static std::vector<float> parse_bearings(const YAML::Node& config);
    static Linspace<float>    parse_ranges(const YAML::Node& config);
    static Waveform::Ptr parse_waveform(const YAML::Node& config);
    static BeamDirectivity::Ptr parse_beamsteering(const YAML::Node& config);
};

} //namespace simulation
} //namespace rtac


#endif //_DEF_RTAC_SIMULATION_SENSOR_INFO_FACTORY_H_
