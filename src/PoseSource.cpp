#include <rtac_simulation/PoseSource.h>

#include <fstream>

namespace rtac { namespace simulation {

PoseSourceList::Ptr PoseSourceList::CreateFromCSV(const std::string& path,
                                                  unsigned int skipLines,
                                                  char delimiter)
{
    std::deque<Pose> poses;

    std::ifstream file(path);
    if(!file.is_open()) {
        throw FileError() << " : could not open '" << path << '\'';
    }
    std::string line;

    // skiping lines
    for(unsigned int i = 0; i < skipLines && std::getline(file, line); i++);
    
    // parsing poses
    while(std::getline(file, line)) {
        if(line[0] == '#') continue;
        poses.push_back(Pose::decode_string(line, delimiter));
    }

    return PoseSourceList::Create(poses);
}

} //namespace simulation
} //namespace rtac

