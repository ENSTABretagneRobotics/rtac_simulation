#include <rtac_simulation/factories/utilities.h>

#include <sstream>
#include <cstring>
// using experimental filesystem because of bug in gcc8
#include <experimental/filesystem>

#include <rtac_base/files.h>
#include <rtac_base/Exception.h>

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

namespace rtac { namespace simulation {

namespace fs = std::experimental::filesystem;

FileFinder::Ptr FileFinder::GlobalInstance = nullptr;

FileFinder::FileFinder(const std::vector<std::string>& searchPaths,
                       const std::string& envVar) :
    searchPaths_(searchPaths)
{
    this->load_rtac_config_paths(envVar);
}

FileFinder::Ptr FileFinder::Get(const std::vector<std::string>& searchPaths,
                                const std::string& envVar)
{
    if(!GlobalInstance) {
        #ifdef RTAC_TEST_CONFIG_PATH
            std::vector<std::string> sPaths(searchPaths);
            sPaths.push_back(std::string(RTAC_TEST_CONFIG_PATH));
            GlobalInstance = Ptr(new FileFinder(sPaths, envVar));
        #else
            GlobalInstance = Ptr(new FileFinder(searchPaths, envVar));
        #endif
    }
    return GlobalInstance;
}

void FileFinder::load_rtac_config_paths(const std::string& envVar)
{
    const char* pathList = std::getenv(envVar.c_str());
    if(!pathList)
        return;

    std::istringstream iss(pathList);
    std::string token;
    while(std::getline(iss, token, ':')) {
        this->add_search_path(token);
    }
}

bool FileFinder::add_search_path(const std::string& path)
{
    if(path.size() == 0) {
        return false;
    }
    fs::path p = fs::absolute(path);
    if(!fs::exists(p)) {
        std::cerr << "FileFinder : path '" << p
                  << "' does not exists." << std::endl;
        return false;
    }
    if(fs::is_regular_file(p)) {
        p = p.parent_path();
    }
    if(!fs::is_directory(p)) {
        std::cerr << "FileFinder : path '" << p
                  << "' is not a directory." << std::endl;
        return false;
    }
    searchPaths_.push_back(p.string());
    return true;
}

std::string FileFinder::find_one(const std::string& filename)
{
    std::string regex = ".*" + filename;
    for(const auto& searchPath : searchPaths_) {
        auto path = files::find_one(regex, searchPath);
        if(path != files::NotFound) {
            return path;
        }
    }
    return files::NotFound;
}

std::vector<std::string> FileFinder::find(const std::string& filename)
{
    std::vector<std::string> res;

    std::string regex = ".*" + filename;
    for(const auto& searchPath : searchPaths_) {
        for(const auto& path : files::find(regex, searchPath)) {
            res.push_back(path);
        }
    }

    return res;
}

/**
 * Loads bearing data from a .csv file. Delimiters MUST be commas.
 */
std::vector<float> parse_bearings_from_csv(const std::string& filename)
{
    std::vector<float> res;

    std::ifstream f(filename);
    if(!f.is_open()) {
        throw FileError(filename) << " : could not open.";
    }
    std::string token;
    if(!std::getline(f, token, ',')) {
        throw FileError(filename) << " : invalid format";
    }

    float scaling = 1.0f;
    if(token == "deg") {
        scaling = M_PI / 180.0f;
    }
    else if(token != "rad") {
        try {
            res.push_back(std::stof(token));
        }
        catch(const std::invalid_argument&) {
            throw FileError(filename)
                << " : invalid format (first item should be either 'deg', 'rad'"
                << " or a decimal value for the first bearing. Got : "
                << token << ')';
        }
    }
    try {
        while(std::getline(f, token, ',')) {
            res.push_back(scaling*std::stof(token));
        }
    }
    catch(const std::invalid_argument) {
        throw FileError(filename)
            << " : invalid format (could not parse "
            << token << " as a number).";
    }
    return res;
}

Directivity::Ptr parse_directivity(const YAML::Node& config)
{
    if(!config) {
        throw ConfigError() << " : Invalid directivity node.";
    }

    float scaling = parse_distance_unit(config["unit"]);


    std::string type = config["type"].as<std::string>();
    if(type == "rectangle")
    {
        std::string baffleMode = "";
        if(auto bMode = config["baffleMode"]) {
            baffleMode = bMode.as<std::string>();
        }
        float wavelength = config["globals"]["sound-celerity"].as<float>()
                         / config["globals"]["frequency"].as<float>();
        return Directivity::rectangle_antenna(scaling*config["width"].as<float>(),
                                              scaling*config["height"].as<float>(),
                                              wavelength, baffleMode);
    }
    else if(type == "uniform")
    {
        float amplitude = 1.0;
        if(auto ampNode = config["amplitude"]) {
            amplitude = ampNode.as<float>();
        }
        return Directivity::make_uniform(amplitude);
    }
    else
    {
        std::ostringstream oss;
        oss << "Unsupported bearing config type : " << type;
        throw std::runtime_error(oss.str());
    }
    // should never reach here
    return nullptr;
}

/**
 * Parse a distance scaling from a 'unit' yaml node.
 * 
 * This returns a scaling value to convert in meters distance value read in the
 * configuration file in meters. (in RTAC all scalars are in SI units, but
 * parameters and display values may be in other units).
 *
 * Currently supported distance units are mm,cm,m.
 */
float parse_distance_unit(const YAML::Node& config)
{
    if(!config)
        return 1.0f;
    auto unit = config.as<std::string>();
    if(unit == "cm")
        return 0.01;
    else if(unit == "mm")
        return 0.001;
    else if(unit != "m") {
        throw ConfigError() << " : unsupported distance unit : '" << unit << "'";
    }
    return 1.0f;
}

/**
 * Parse an angle scaling from a 'unit' yaml node.
 * 
 * This returns a scaling value to convert values read in the configuration
 * file in radians (in RTAC all scalars are in SI units, but parameters and
 * display values may be in other units).
 *
 * Currently supported distance units are deg,rad.
 */
float parse_angle_unit(const YAML::Node& config)
{
    if(!config)
        return 1.0f;
    auto unit = config.as<std::string>();
    if(unit == "deg")
        return M_PI / 180.0f;
    else if(unit != "rad") {
        throw ConfigError() << " : unsupported angle unit : '" << unit << "'";
    }
    return 1.0f;
}

} //namespace simulation
} //namespace rtac

std::ostream& operator<<(std::ostream& os, const rtac::simulation::FileFinder& ff)
{
    os << "FileFinder :";
    for(const auto& p : ff.search_paths()) {
        os << "\n- " << p;
    }
    return os;
}
