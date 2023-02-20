#include <iostream>
using namespace std;

#include <rtac_simulation/factories/PSFFactory.h>
using namespace rtac::simulation;

int main()
{
    auto filename = std::string(RTAC_CONFIG_FILES_PATH) + "point_spread_function_2d.yaml";
    std::cout << "config file : " << filename << std::endl;

    YAML::Node config = YAML::LoadFile(filename);

    auto psfFactory = PSFFactory::Create(config);

    return 0;
}


