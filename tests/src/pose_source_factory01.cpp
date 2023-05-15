#include <iostream>
using namespace std;

#include <yaml-cpp/yaml.h>

#include <rtac_simulation/PoseSource.h>
#include <rtac_simulation/factories/utilities.h>
#include <rtac_simulation/factories/PoseSourceFactory.h>
using namespace rtac::simulation;

int main()
{
    auto poseSource = PoseSourceFactory::Make("pose_source.yaml");
    if(auto p = std::dynamic_pointer_cast<PoseSourceStatic>(poseSource))
    {
        std::cout << "PoseSourceStatic : " << p->next_pose() << std::endl;
    }
    else if(auto poses = std::dynamic_pointer_cast<PoseSourceList>(poseSource)) {
        std::cout << "PoseSourceList (size : " << poses->size() << ")\n";
        while(poses->remaining() > 0) {
            std::cout << poses->next_pose() << std::endl;
        }
    }

    return 0;
}
