#ifndef _DEF_RTAC_SIMULATION_FACTORIES_UTILITIES_H_
#define _DEF_RTAC_SIMULATION_FACTORIES_UTILITIES_H_

#include <memory>
#include <vector>
#include <iostream>

#include <yaml-cpp/yaml.h>

#include <rtac_base/Exception.h>

#include <rtac_simulation/Directivity.h>

namespace rtac { namespace simulation {

struct ConfigError : public Exception
{
    ConfigError() : Exception("RTAC_CONFIG_ERROR") {}
};

class FileFinder
{
    public:

    using Ptr      = std::shared_ptr<FileFinder>;
    using ConstPtr = std::shared_ptr<const FileFinder>;

    static const char* const NotFound;

    protected:

    static Ptr GlobalInstance;

    std::vector<std::string> searchPaths_;

    FileFinder(const std::vector<std::string>& searchPaths = {},
               const std::string& envVar = "RTAC_CONFIG_PATH");

    public:

    static Ptr Get(const std::vector<std::string>& searchPaths = {},
                   const std::string& envVar = "RTAC_CONFIG_PATH");
    static std::string FindOne(const std::string& filename);

    const std::vector<std::string>& search_paths() const { return searchPaths_; }

    void load_rtac_config_paths(const std::string& envVar);
    bool add_search_path(const std::string& path);

    std::string find_one(const std::string& filename);
    std::vector<std::string> find(const std::string& filename);
};

HostVector<float> parse_bearings_from_csv(const std::string& filename);
Directivity::Ptr   parse_directivity(const YAML::Node& config, float wavelength = -1);

float parse_distance_unit(const YAML::Node& config);
float parse_angle_unit(const YAML::Node& config);

} //namespace simulation
} //namespace rtac

std::ostream& operator<<(std::ostream& os, const rtac::simulation::FileFinder& ff);

#endif //_DEF_RTAC_SIMULATION_FACTORIES_UTILITIES_H_
