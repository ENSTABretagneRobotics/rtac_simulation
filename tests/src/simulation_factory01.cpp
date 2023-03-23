#include <iostream>
using namespace std;

#include <rtac_simulation/factories/SimulationFactory.h>
using namespace rtac::simulation;

int main()
{
    auto filename = FileFinder::Get()->find_one("simulation1.yaml");
    std::cout << "Simulation config file : " << filename << std::endl;
    auto simulation = SimulationFactory::Make1(filename);
    return 0;
}
