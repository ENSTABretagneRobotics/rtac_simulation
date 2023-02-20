#include <rtac_simulation/factories/PSFFactory.h>

namespace rtac { namespace simulation {

PSFFactory::PSFFactory(const YAML::Node& config)
{
    std::cout << config << std::endl;
}

} //namespace simulation
} //namespace rtac
