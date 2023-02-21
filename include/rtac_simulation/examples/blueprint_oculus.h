#ifndef _DEF_RTAC_SIMULATION_EXAMPLES_BLUEPRINT_OCULUS_H_
#define _DEF_RTAC_SIMULATION_EXAMPLES_BLUEPRINT_OCULUS_H_

#include <memory>

#include <rtac_simulation/Directivity.h>
#include <rtac_simulation/PolarKernel2D.h>
#include <rtac_simulation/PolarTarget2D.h>
#include <rtac_simulation/PolarReceiver2D.h>

#include <rtac_simulation/ReductionKernel.h>
#include <rtac_simulation/SensorModel.h>

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

class OculusSensor
{
    public:

    using Ptr      = std::shared_ptr<OculusSensor>;
    using ConstPtr = std::shared_ptr<const OculusSensor>;

    //using SensorModel = SensorModel2D<Complex<float>, float>;
    //static Kernel2D<float> make_lf_kernel(float rangeResolution);
    //static Kernel2D<float> make_hf_kernel(float rangeResolution);

    using SensorModel = SensorModel2D_2<Complex<float>>;
    static PointSpreadFunction2D::Ptr make_lf_kernel(float rangeResolution);
    static PointSpreadFunction2D::Ptr make_hf_kernel(float rangeResolution);

    enum FrequencyMode {
        ModeNone,
        LowFrequency,
        HighFrequency
    };

    protected:

    FrequencyMode    currentMode_;
    SensorModel::Ptr lfSensor_;
    SensorModel::Ptr hfSensor_;

    OculusSensor() : currentMode_(ModeNone) {}

    public:

    static Ptr Create() { return Ptr(new OculusSensor()); }

    SensorModel::Ptr sensor() const {
        switch(currentMode_) {
            default:            return nullptr;   break;
            case LowFrequency:  return lfSensor_; break;
            case HighFrequency: return hfSensor_; break;
        }
    }

    bool needs_update(FrequencyMode requestedMode, 
                      float minRange, float maxRange,
                      unsigned int nRanges, unsigned int nBearings);
    bool reconfigure(FrequencyMode Mode,
                     float minRange, float maxRange,
                     unsigned int nRanges,
                     const std::vector<float>& bearings,
                     bool forceReconfigure = false);
    
    bool reconfigure(const OculusSimplePingResult& metadata,
                     const uint8_t* data,
                     bool forceReconfigure = false);
};

//#endif //RTAC_OCULUS_DRIVER

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_EXAMPLES_BLUEPRINT_OCULUS_H_
