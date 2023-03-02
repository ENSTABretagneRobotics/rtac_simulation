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

    std::vector<std::string> searchPaths_;

    FileFinder(const std::vector<std::string>& searchPaths = {},
               const std::string& envVar = "RTAC_CONFIG_PATH");

    void load_rtac_config_paths(const std::string& envVar);

    public:

    static Ptr Create(const std::vector<std::string>& searchPaths = {},
                      const std::string& envVar = "RTAC_CONFIG_PATH")
    {
        return Ptr(new FileFinder(searchPaths, envVar));
    }

    const std::vector<std::string>& search_paths() const { return searchPaths_; }

    bool add_search_path(const std::string& path);

    std::string find_one(const std::string& filename);
    std::vector<std::string> find(const std::string& filename);
};

//FileFinder::Ptr file_finder();
//void add_search_path(const std::string& path);
//
//std::string find_one_config_file(const std::string& filename);
//std::vector<std::string> find_one_config_file(const std::string& filename);

} //namespace simulation
} //namespace rtac

std::ostream& operator<<(std::ostream& os, const rtac::simulation::FileFinder& ff);

#endif //_DEF_RTAC_SIMULATION_FACTORIES_UTILITIES_H_
