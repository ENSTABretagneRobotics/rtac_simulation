#include <iostream>
using namespace std;

#include <rtac_simulation/factories/utilities.h>
#include <rtac_simulation/factories/SensorInfoFactory.h>
using namespace rtac::simulation;

int main()
{
    auto finder = FileFinder::Get({std::string(RTAC_TEST_CONFIG)});
    auto filename = finder->find_one(".*sensor_info01.yaml");
    std::cout << "config file : " << filename << std::endl;

    auto sensorInfo = SensorInfoFactory2D::Make(YAML::LoadFile(filename));

    return 0;
}



