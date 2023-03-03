#ifndef _DEF_RTAC_SIMULATION_FACTORIES_UTILITIES_H_
#define _DEF_RTAC_SIMULATION_FACTORIES_UTILITIES_H_

#include <memory>
#include <vector>
#include <iostream>

namespace rtac { namespace simulation {

class FileFinder
{
    public:

    using Ptr      = std::shared_ptr<FileFinder>;
    using ConstPtr = std::shared_ptr<const FileFinder>;

    protected:

    static Ptr GlobalInstance;

    std::vector<std::string> searchPaths_;

    FileFinder(const std::vector<std::string>& searchPaths = {},
               const std::string& envVar = "RTAC_CONFIG_PATH");

    public:

    static Ptr Get(const std::vector<std::string>& searchPaths = {},
                   const std::string& envVar = "RTAC_CONFIG_PATH");

    const std::vector<std::string>& search_paths() const { return searchPaths_; }

    void load_rtac_config_paths(const std::string& envVar);
    bool add_search_path(const std::string& path);

    std::string find_one(const std::string& filename);
    std::vector<std::string> find(const std::string& filename);
};

std::vector<float> load_csv_bearings(const std::string& filename);

} //namespace simulation
} //namespace rtac

std::ostream& operator<<(std::ostream& os, const rtac::simulation::FileFinder& ff);

#endif //_DEF_RTAC_SIMULATION_FACTORIES_UTILITIES_H_
