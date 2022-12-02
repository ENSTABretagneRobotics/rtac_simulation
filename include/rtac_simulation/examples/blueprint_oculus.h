#ifndef _DEF_RTAC_SIMULATION_EXAMPLES_BLUEPRINT_OCULUS_H_
#define _DEF_RTAC_SIMULATION_EXAMPLES_BLUEPRINT_OCULUS_H_

#include <memory>

#include <rtac_simulation/Directivity.h>
#include <rtac_simulation/PolarKernel2D.h>
#include <rtac_simulation/PolarTarget2D.h>
#include <rtac_simulation/PolarReceiver2D.h>

#include <narval_oculus/Oculus.h>

namespace rtac { namespace simulation {

class OculusReceiver : public PolarReceiver2D<float>
{
    public:

    using Ptr      = std::shared_ptr<OculusReceiver>;
    using ConstPtr = std::shared_ptr<const OculusReceiver>;

    using Kernel = PolarReceiver2D<float>::Kernel;
    using Target = PolarReceiver2D<float>::Target;

    static Kernel::ConstPtr make_lf_kernel(float rangeResolution);
    static Kernel::ConstPtr make_hf_kernel(float rangeResolution);

    enum FrequencyMode {
        ModeNone,
        LowFrequency,
        HighFrequency
    };

    protected:

    FrequencyMode         currentMode_;

    Directivity::ConstPtr lfDirectivity_;
    Kernel::ConstPtr      lfKernel_;
    Target::Ptr           lfTarget_;

    Directivity::ConstPtr hfDirectivity_;
    Kernel::ConstPtr      hfKernel_;
    Target::Ptr           hfTarget_;
    
    OculusReceiver();

    public:

    static Ptr Create() { return Ptr(new OculusReceiver()); }

    bool needs_update(FrequencyMode requestedMode, float maxRange,
                      unsigned int nRanges, unsigned int nBearings);
    bool reconfigure(FrequencyMode requestedMode,
                     float maxRange, unsigned int nRanges,
                     const std::vector<float>& bearings,
                     bool forceReconfigure = false);
    
    //#if defined(RTAC_OCULUS_DRIVER) || defined(RTAC_OCULUS_DEPRECATED)
    bool reconfigure(const OculusSimplePingResult& metadata,
                     const uint8_t* data,
                     bool forceReconfigure = false);
    //#endif //RTAC_OCULUS_DRIVER
};

//#if defined(RTAC_OCULUS_DRIVER) || defined(RTAC_OCULUS_DEPRECATED)

inline bool OculusReceiver::reconfigure(const OculusSimplePingResult& metadata,
                                        const uint8_t* data,
                                        bool forceReconfigure)
{
    FrequencyMode requestedMode = ModeNone;
    if(metadata.fireMessage.masterMode == 1) {
        requestedMode      = LowFrequency;
        this->target_      = lfTarget_;
        this->psf_         = lfKernel_;
        this->directivity_ = lfDirectivity_;
    }
    else if(metadata.fireMessage.masterMode == 2) {
        requestedMode      = HighFrequency;
        this->target_      = hfTarget_;
        this->psf_         = hfKernel_;
        this->directivity_ = hfDirectivity_;
    }
    else {
        std::cerr << "OculusReceiver : Invalid requested frequency mode : "
                  << (int)metadata.fireMessage.masterMode 
                  << ". Ignoring reconfiguration." << std::endl;
        return false;
    }

    if(forceReconfigure || this->needs_update(requestedMode, 
                                              metadata.fireMessage.range,
                                              metadata.nRanges,
                                              metadata.nBeams))
    {
        std::vector<float> bearingData(metadata.nBeams);
        auto bearings = (const int16_t*)(data + sizeof(OculusSimplePingResult));
        for(int i = 0; i < metadata.nBeams; i++) {
            bearingData[i] = (0.01f*M_PI/180.0f) * bearings[i];
        }
        return this->reconfigure(requestedMode, metadata.fireMessage.range,
                                 metadata.nRanges, bearingData, true);
    }

    return false;
}

//#endif //RTAC_OCULUS_DRIVER

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_EXAMPLES_BLUEPRINT_OCULUS_H_
