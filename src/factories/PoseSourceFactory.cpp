#include <rtac_simulation/factories/PoseSourceFactory.h>

#include <rtac_simulation/factories/utilities.h>
#include <rtac_base/files.h>

namespace rtac { namespace simulation {

PoseSource::Ptr PoseSourceFactory::Make(const YAML::Node& config)
{
    if(!config || !config["type"]) {
        throw ConfigError() << " : invalid config for PoseSource";
    }

    std::string type = config["type"].as<std::string>();
    if(type == "static") {
        return PoseSourceFactory::parse_static(config);
    }
    else if(type == "csv") {
        auto path = FileFinder::Get()->find_one(config["path"].as<std::string>());
        if(path == files::NotFound) {
            throw ConfigError() << " : could not find pose .csv file '"
                                << config["path"].as<std::string>() << '\'';
        }
        return PoseSourceList::CreateFromCSV(path);
            
    }
    else {
        throw ConfigError() << " : unsupported type for pose source '"
                            << type << '\'';
    }
}

PoseSource::Ptr PoseSourceFactory::Make(const std::string& configPath)
{
    auto path = FileFinder::Get()->find_one(configPath);
    if(path == files::NotFound) {
        throw ConfigError() << " : could not find '" << configPath << '\'';
    }
    return PoseSourceFactory::Make(YAML::LoadFile(path));
}

PoseSource::Ptr PoseSourceFactory::parse_static(const YAML::Node& config)
{
    if(auto node = config["str"]) {
        return PoseSourceStatic::Create(Pose::decode_string(node.as<std::string>()));
    }

    bool configValid = false;
    Pose p = Pose::Identity();

    if(auto tnode = config["t"]) {
        p.x() = tnode[0].as<float>();
        p.y() = tnode[1].as<float>();
        p.z() = tnode[2].as<float>();
        configValid = true;
    }

    if(auto qnode = config["q"]) {
        Pose::Quat q;
        q.w() = qnode[0].as<float>();
        q.x() = qnode[1].as<float>();
        q.y() = qnode[2].as<float>();
        q.z() = qnode[3].as<float>();
        p.set_orientation(q);
        configValid = true;
    }
    else if (auto hnode = config["h"]) {
        Pose::Mat4 h;
        h(0,0) = hnode[0].as<float>();
        h(0,1) = hnode[1].as<float>();
        h(0,2) = hnode[2].as<float>();
        h(0,3) = hnode[3].as<float>();

        h(1,0) = hnode[4].as<float>();
        h(1,1) = hnode[5].as<float>();
        h(1,2) = hnode[6].as<float>();
        h(1,3) = hnode[7].as<float>();

        h(2,0) = hnode[8].as<float>();
        h(2,1) = hnode[9].as<float>();
        h(2,2) = hnode[10].as<float>();
        h(2,3) = hnode[11].as<float>();

        h(3,0) = 0.0; h(3,1) = 0.0; h(3,2) = 0.0; h(3,3) = 1.0;
        configValid = true;
    }

    if(!configValid) {
        throw ConfigError() << " : invalid config for static PoseSource, " << config;
    }

    return PoseSourceStatic::Create(p);
}

} //namespace simulation
} //namespace rtac
