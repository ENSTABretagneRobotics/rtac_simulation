#include <rtac_simulation/factories/SensorInfoFactory.h>

#include <rtac_simulation/SensorInstance1D.h>

namespace rtac { namespace simulation {

SensorInstance::Ptr SensorFactory::Make(const YAML::Node& config)
{
    auto sensorType = config["type"];
    if(!sensorType) {
        throw ConfigError() << " : sensor config has no type.";
    }
    std::string type = sensorType.as<std::string>();
    if(type == "file") {
        return Make(FileFinder::Get()->find_one(config["path"].as<std::string>()));
    }
    
    std::string outputMode = "complex";
    if(auto node = config["output-node"]) {
        outputMode = node.as<std::string>();
    }

    SensorInfo::Ptr info;
    if(type == "front-scan") {
        info = SensorInfoFactory::Make_FrontScanInfo(config);
        if(outputMode == "complex") {
            //return SensorInstance2D_Complex::Create(info);
            return SensorInstance2D_2<Complex<float>>::Create(info);
        }
        else if(outputMode == "real") {
            throw ConfigError() << " : real sensor output mode not supported yet.";
        }
        else {
            throw ConfigError() << " : unsupported output mode (" << outputMode << ")";
        }
    }
    else if(type == "single-beam") {
        info = SensorInfoFactory::Make_SingleBeamInfo(config);
        if(outputMode == "complex") {
            return SensorInstance1D_Complex::Create(info);
        }
        else if(outputMode == "real") {
            throw ConfigError() << " : real sensor output mode not supported yet.";
        }
        else {
            throw ConfigError() << " : unsupported output mode (" << outputMode << ")";
        }
    }
    else {
        throw ConfigError() << " :  unsupported sensor type '" << type << "'";
    }
}

SensorInstance::Ptr SensorFactory::Make(const std::string& configPath)
{
    return SensorFactory::Make(YAML::LoadFile(configPath));
}

SensorInfo::Ptr SensorInfoFactory::Make(const YAML::Node& config)
{
    auto sensorType = config["type"];
    if(!sensorType) {
        throw ConfigError() << " : sensor config has no type.";
    }
    
    std::string type = sensorType.as<std::string>();
    if(type == "front-scan") {
        return Make_FrontScanInfo(config);
    }
    else if(type == "single-beam") {
        return Make_SingleBeamInfo(config);
    }
    else {
        throw ConfigError() << " :  unsupported sensor type '" << type << "'";
    }
}

SensorInfo::Ptr SensorInfoFactory::Make(const std::string& configPath)
{
    return SensorInfoFactory::Make(YAML::LoadFile(configPath));
}

SensorInfo2D::Ptr SensorInfoFactory::Make_FrontScanInfo(const YAML::Node& config)
{
    auto samplingNode = config["sampling"];
    if(!samplingNode) {
        throw ConfigError() << " : No 'sampling' node";
    }
    HostVector<float> bearings = parse_bearings(samplingNode["bearings"]);
    Linspace<float>    ranges   = parse_ranges(samplingNode["ranges"]);
    auto directivity = parse_directivity(config["directivity"]);
    auto waveform = parse_waveform(config["waveform"]);
    auto beam = parse_beamsteering(config["beamsteering"]);

    return SensorInfo2D::Create(bearings, ranges, waveform, beam, directivity);
}

SensorInfo::Ptr SensorInfoFactory::Make_SingleBeamInfo(const YAML::Node& config)
{
    auto samplingNode = config["sampling"];
    if(!samplingNode) {
        throw ConfigError() << " : No 'sampling' node";
    }
    Linspace<float>  ranges = parse_ranges(samplingNode["ranges"]);
    auto directivity = parse_directivity(config["directivity"]);
    auto waveform    = parse_waveform(config["waveform"]);

    return SensorInfo::Create(ranges, waveform, directivity);
}

Linspace<float> SensorInfoFactory::parse_ranges(const YAML::Node& config)
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


HostVector<float> SensorInfoFactory::parse_bearings(const YAML::Node& config)
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

Waveform::Ptr SensorInfoFactory::parse_waveform(const YAML::Node& config)
{
    if(!config) {
        throw ConfigError() << " : Invalid 'waveform' node.";
    }

    unsigned int oversampling = 8;
    if(auto node = config["oversampling"]) {
        oversampling = node.as<unsigned int>();
    }

    std::string type = config["type"].as<std::string>();
    if(type == "sine") {
        float frequency = config["frequency"].as<float>();
        if(auto durationMode = config["duration-mode"]) {
            std::string mode = durationMode.as<std::string>();
            if(mode == "fixed") {
                return Waveform_Sine::Create(frequency,
                                             config["duration"].as<float>(),
                                             true,
                                             oversampling);
            }
            else if(mode != "adaptive") {
                std::cerr << "Got invalid duration mode : '" << mode
                          << "'. Falling back to 'adaptive'" << std::endl;
            }
        }
        return Waveform_Sine::Create(frequency, 10 / frequency, false, oversampling);
    }
    else {
        throw ConfigError() << " : unsupported waveform type : '" << type << "'";
    }
}

BeamDirectivity::Ptr SensorInfoFactory::parse_beamsteering(const YAML::Node& config)
{
    if(!config) {
        throw ConfigError() << " : Invalid 'beamsteering' node.";
    }

    unsigned int oversampling = 8;
    if(auto node = config["oversampling"]) {
        oversampling = node.as<unsigned int>();
    }

    std::string type = config["type"].as<std::string>();
    if(type == "sinc")
    {
        float span       = config["span"].as<float>();
        float resolution = config["resolution"].as<float>();
        float scaling = 1.0f;
        if(auto unitNode = config["unit"]) 
        {
            std::string unit = unitNode.as<std::string>();
            if(unit == "deg") {
                scaling = M_PI / 180.0f;
            }
            else if(unit != "rad"){
                std::cerr << "Invalid unit : '" << unit
                          << "'. defaulting to 'rad'.";
            }
        }
        return BeamDirectivity_Sinc::Create(scaling*span, scaling*resolution, oversampling);
    }
    else {
        throw ConfigError() << " : unsupported beamsteering type : '" << type << "'";
    }
}

} //namespace simulation
} //namespace rtac


