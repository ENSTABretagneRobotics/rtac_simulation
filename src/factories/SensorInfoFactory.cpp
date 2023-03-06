#include <rtac_simulation/factories/SensorInfoFactory.h>

namespace rtac { namespace simulation {

SensorInfo2D::Ptr SensorInfoFactory2D::Make(const YAML::Node& config)
{
    auto samplingNode = config["sampling"];
    if(!samplingNode) {
        throw ConfigError() << " : No 'sampling' node";
    }
    std::vector<float> bearings = parse_bearings(samplingNode["bearings"]);
    Linspace<float>    ranges   = parse_ranges(samplingNode["ranges"]);
    auto directivity = parse_directivity(config["directivity"]);

    return nullptr;
}

Linspace<float> SensorInfoFactory2D::parse_ranges(const YAML::Node& config)
{
    if(!config) {
        throw ConfigError() << " : Invalid ranges node.";
    }

    float rangeMin = 0.0f;
    auto globals = config["globals"];
    if(globals && globals["range-min"]) {
        rangeMin = globals["range-min"].as<float>();
    }

    auto size = config["size"];
    if(!size) {
        throw ConfigError() << " : invalid ranges node (must have a 'size' field)";
    }
    auto rangeMax = config["range-max"];
    if(!rangeMax) {
        throw ConfigError() << " : invalid ranges node (must have a 'range-max' field)";
    }

    float scaling = 1.0f;
    auto unit = config["unit"];
    if(unit) {
        std::string unitStr = unit.as<std::string>();
        if(unitStr == "cm") scaling = 0.01f;
        else if(unitStr == "mm") scaling = 0.001f;
        else if(unitStr != "m") {
            std::cerr << "Invalid range unit string ("
                      << unitStr << "). Falling back to meters";
        }
    }
    
    // no scaling on rangeMin. Unit is fixed to meters for now
    return Linspace<float>(rangeMin,
                           scaling*rangeMax.as<float>(),
                           size.as<unsigned int>());
}


std::vector<float> SensorInfoFactory2D::parse_bearings(const YAML::Node& config)
{
    if(!config) {
        throw ConfigError() << " : Invalid bearings node.";
    }
    std::string type = config["type"].as<std::string>();
    if(type == "csv") {
        auto filename = config["path"].as<std::string>();
        return parse_bearings_from_csv(FileFinder::Get()->find_one(filename));
    }
    else {
        std::ostringstream oss;
        oss << "Unsupported bearing config type : " << type;
        throw std::runtime_error(oss.str());
    }
}

Directivity::Ptr SensorInfoFactory2D::parse_directivity(const YAML::Node& config)
{
    if(!config) {
        throw ConfigError() << " : Invalid directivity node.";
    }

    std::string baffleMode = "";
    if(auto bMode = config["baffleMode"]) {
        baffleMode = bMode.as<std::string>();
    }

    float scaling = 1.0f;
    if(auto unitNode = config["unit"]) {
        std::string unit = unitNode.as<std::string>();
        if(unit == "mm")
            scaling = 0.001;
        else if(unit == "cm")
            scaling = 0.01;
        else if(unit != "m") {
            std::cerr << "Invalid length unit '" << unit
                      << "'. Defautling to meters." << std::endl;
        }
    }

    float wavelength = config["globals"]["sound-celerity"].as<float>()
                     / config["globals"]["frequency"].as<float>();

    std::string type = config["type"].as<std::string>();
    if(type == "rectangle") {
        return Directivity::rectangle_antenna(scaling*config["width"].as<float>(),
                                              scaling*config["height"].as<float>(),
                                              wavelength, baffleMode);
    }
    else {
        std::ostringstream oss;
        oss << "Unsupported bearing config type : " << type;
        throw std::runtime_error(oss.str());
    }
}

} //namespace simulation
} //namespace rtac


