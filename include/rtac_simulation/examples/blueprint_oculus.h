#ifndef _DEF_RTAC_SIMULATION_EXAMPLES_BLUEPRINT_OCULUS_H_
#define _DEF_RTAC_SIMULATION_EXAMPLES_BLUEPRINT_OCULUS_H_

#include <memory>

#include <rtac_simulation/Directivity.h>
#include <rtac_simulation/PolarKernel2D.h>
#include <rtac_simulation/PolarTarget2D.h>
#include <rtac_simulation/PolarReceiver2D.h>

namespace rtac { namespace simulation { namespace oculus {

class OculusReceiverLF : public PolarReceiver2D<float>
{
    public:

    using Ptr      = std::shared_ptr<OculusReceiverLF>;
    using ConstPtr = std::shared_ptr<const OculusReceiverLF>;

    using Kernel = PolarReceiver2D<float>::Kernel;
    using Target = PolarReceiver2D<float>::Target;

    static Directivity::ConstPtr make_directivity();
    static Kernel::ConstPtr      make_kernel(float rangeResolution);

    protected:
    
    OculusReceiverLF();

    public:

    static Ptr Create() { return Ptr(new OculusReceiverLF()); }

    void update_target(float maxRange, unsigned int nRanges,
                       const std::vector<float>& bearings);
};


} //namespace examples
} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_EXAMPLES_BLUEPRINT_OCULUS_H_
