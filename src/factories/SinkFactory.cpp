#include <rtac_simulation/factories/SinkFactory.h>

#include <rtac_simulation/sinks/Sink.h>
#include <rtac_simulation/sinks/FileSink.h>

#ifdef RTAC_SIMULATION_DISPLAY_ENABLED
#include <rtac_simulation/sinks/DisplaySink.h>
#endif //RTAC_SIMULATION_DISPLAY_ENABLED

#include <rtac_simulation/factories/utilities.h>

namespace rtac { namespace simulation {

Sink::Ptr SinkFactory::Make(const YAML::Node& config)
{
    if(!config || !config["type"]) {
        throw ConfigError() << " : invalid config for Sink";
    }

    std::string name = "Sink";
    if(auto node = config["name"]) {
        name = node.as<std::string>();
    }

    std::string type = config["type"].as<std::string>();
    if(type == "file-sink")
    {
        if(!config["path"]) {
            throw ConfigError() << " : invalid format for file-sink config, "
                                << "Must contain a 'path' field";
        }
        std::string path = config["path"].as<std::string>();
        bool overwrite = false;
        if(auto node = config["overwrite"]) {
            overwrite = node.as<bool>();
        }
        return FileSink::Create(path, overwrite, name);
    }
    else if(type == "display-sink") {
        return DisplaySink::Create(name);
    }
    else {
        throw ConfigError() << " : unsupported sink type '" << type << "'";
    }
}

Sink::Ptr SinkFactory::Make(const std::string& configPath)
{
    return SinkFactory::Make(YAML::LoadFile(configPath));
}

} //namespace simulation
} //namespace rtac


