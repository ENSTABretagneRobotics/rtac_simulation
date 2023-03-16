#include <rtac_simulation/factories/EmitterFactory.h>

#include <rtac_simulation/EmitterGL.h>

namespace rtac { namespace simulation {

EmitterBase::Ptr EmitterFactory::Make(const YAML::Node& config)
{
    if(!config) {
        throw ConfigError() << " : Invalid emitter node.";
    }
    auto typeNode = config["type"];
    if(!typeNode) {
        throw ConfigError() << " : no 'type' node in emitter config.";
    }
    auto type = typeNode.as<std::string>();

    if(type == "emitter_optix") {
        return MakeEmitterOptix(config);
    }
    else if(type == "emitter_gl") {
        return MakeEmitterGL(config);
    }
    else {
        throw ConfigError() << " : unsupported emitter type '" << type << "'";
    }

    // should never happen
    return nullptr;
}

EmitterBase::Ptr EmitterFactory::MakeEmitterOptix(const YAML::Node& config)
{
    if(!config) {
        throw ConfigError() << " : Invalid emitter node.";
    }

    auto directivity = parse_directivity(config["directivity"]);

    auto rayNode = config["ray-config"];
    if(!rayNode) {
        throw ConfigError() << " : no 'ray-factory' node in emitter config.";
    }
    auto rayType = rayNode["type"].as<std::string>();
    if(rayType == "polar") {
        auto resolution        = rayNode["resolution"].as<float>();
        auto bearingAperture   = rayNode["bearing-aperture"].as<float>();
        auto elevationAperture = rayNode["elevation-aperture"].as<float>();
        float scaling = parse_angle_unit(rayNode["unit"]);
        scaling *= 180.0 / M_PI; // Emitter::Create input is in degrees for now
        return Emitter::Create(scaling*resolution,
                               scaling*bearingAperture,
                               scaling*elevationAperture,
                               directivity);
    }
    else {
        throw ConfigError() << " : unsupported ray-config type (" << rayType << ").";
    }

    // should never happen
    return nullptr;
}

EmitterBase::Ptr EmitterFactory::MakeEmitterGL(const YAML::Node& config)
{
    if(!config) {
        throw ConfigError() << " : Invalid emitter node.";
    }

    //auto directivity = parse_directivity(config["directivity"]);

    auto rayNode = config["ray-config"];
    if(!rayNode) {
        throw ConfigError() << " : no 'ray-factory' node in emitter config.";
    }
    auto rayType = rayNode["type"].as<std::string>();
    if(rayType == "polar") {
        auto resolution        = rayNode["resolution"].as<float>();
        auto bearingAperture   = rayNode["bearing-aperture"].as<float>();
        auto elevationAperture = rayNode["elevation-aperture"].as<float>();
        float scaling = parse_angle_unit(rayNode["unit"]);
        scaling *= 180.0 / M_PI; // EmitterGL::Create input is in degrees for now
        return EmitterGL::Create(scaling*resolution,
                                 scaling*bearingAperture,
                                 scaling*elevationAperture);
    }
    else {
        throw ConfigError() << " : unsupported ray-config type (" << rayType << ").";
    }

    // should never happen
    return nullptr;
}


EmitterBase::Ptr EmitterFactory::Make(const std::string& configPath)
{
    return EmitterFactory::Make(YAML::LoadFile(configPath));
}

} //namespace simulation
} //namespace rtac
