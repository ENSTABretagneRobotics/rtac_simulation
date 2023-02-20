#ifndef _DEF_RTAC_SIMULATION_FACTORIES_PSF_FACTORY_H_
#define _DEF_RTAC_SIMULATION_FACTORIES_PSF_FACTORY_H_

#include <memory>


#include <yaml-cpp/yaml.h>

#include <rtac_simulation/ReductionKernel.h>

namespace rtac { namespace simulation {

class PSFFactory
{
    public:

    using Ptr      = std::shared_ptr<PSFFactory>;
    using ConstPtr = std::shared_ptr<const PSFFactory>;

    protected:

    PSFFactory(const YAML::Node& config);

    public:

    static Ptr Create(const YAML::Node& config) { return Ptr(new PSFFactory(config)); }
};

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_FACTORIES_PSF_FACTORY_H_
