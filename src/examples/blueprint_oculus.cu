#include <rtac_simulation/examples/blueprint_oculus.h>

#include <rtac_simulation/helpers/receiver_factories.h>

namespace rtac { namespace simulation {

OculusReceiver::Kernel::ConstPtr OculusReceiver::make_lf_kernel(float rangeResolution)
{
    float bearingResolution = 0.6f;
    float bearingSpan       = 130.0f;
    float pulseLength       = 2*rangeResolution;
    //float waveLength        = 0.0012178;
    float waveLength        = 1500.0 / 1.2e6;

    return simple_polar_kernel<float>(bearingResolution, bearingSpan,
                                      pulseLength, waveLength);
}

OculusReceiver::Kernel::ConstPtr OculusReceiver::make_hf_kernel(float rangeResolution)
{
    float bearingResolution = 0.4f;
    float bearingSpan       = 80.0f;
    float pulseLength       = 2*rangeResolution;
    float waveLength        = 1500.0 / 2.1e6;

    return simple_polar_kernel<float>(bearingResolution, bearingSpan,
                                      pulseLength, waveLength);
}

OculusReceiver::OculusReceiver() :
    PolarReceiver2D<float>(nullptr, nullptr, nullptr),
    currentMode_(ModeNone),
    lfDirectivity_(Directivity::from_sinc_parameters(130.0f, 20.0f)),
    hfDirectivity_(Directivity::from_sinc_parameters(80.0f, 12.0f))
{}

bool OculusReceiver::needs_update(FrequencyMode requestedMode, float maxRange,
                                  unsigned int nRanges, unsigned int nBearings)
{
    if(requestedMode != currentMode_) {
        return true;
    }
    if(requestedMode == LowFrequency) {
        if(!lfTarget_ || !lfKernel_
            || lfTarget_->needs_update(0.1f, maxRange, nRanges, nBearings))
        {
            return true;
        }
        if(this->directivity().get() != lfDirectivity_.get()
           || this->target().get()   != lfTarget_.get()
           || this->psf().get()      != lfKernel_.get())
        {
            return true;
        }
    }
    else if(requestedMode == HighFrequency) {
        if(!hfTarget_ || !hfKernel_
            || hfTarget_->needs_update(0.1f, maxRange, nRanges, nBearings))
        {
            return true;
        }
        if(this->directivity().get() != hfDirectivity_.get()
           || this->target().get()   != hfTarget_.get()
           || this->psf().get()      != hfKernel_.get())
        {
            return true;
        }
    }
    return false;
}

bool OculusReceiver::reconfigure(FrequencyMode mode,
                                 float maxRange, unsigned int nRanges,
                                 const std::vector<float>& bearings,
                                 bool forceReconfigure)
{
    std::cout << "reconfigure" << std::endl;
    if(mode == ModeNone) {
        std::cerr << "OculusReceiver : Invalid requested frequency mode. "
                  << "Ignoring reconfiguration." << std::endl;
        return false;
    }

    if(!this->needs_update(mode, maxRange, nRanges, bearings.size()) && !forceReconfigure)
        return false;

    std::vector<float> ranges(nRanges);
    for(int n = 0; n < nRanges; n++) {
        ranges[n] = ((maxRange - 0.1f) * n) / (nRanges - 1) + 0.1f;
    }
    if(mode == LowFrequency)
    {
        if(!lfTarget_) {
            lfTarget_ = PolarTarget2D<Complex<float>>::Create(bearings, ranges);
        }
        else {
            lfTarget_->update(bearings, ranges);
        }
        lfKernel_          = make_lf_kernel(lfTarget_->range_resolution());
        this->target_      = lfTarget_;
        this->psf_         = lfKernel_;
        this->directivity_ = lfDirectivity_;
        currentMode_       = LowFrequency;
        return true;
    }
    else if(mode == HighFrequency)
    {
        if(!hfTarget_) {
            hfTarget_ = PolarTarget2D<Complex<float>>::Create(bearings, ranges);
        }
        else {
            hfTarget_->update(bearings, ranges);
        }
        hfKernel_          = make_hf_kernel(hfTarget_->range_resolution());
        this->target_      = hfTarget_;
        this->psf_         = hfKernel_;
        this->directivity_ = hfDirectivity_;
        currentMode_       = HighFrequency;
    }
    // should never happen
    return false;
}

} //namespace simulation
} //namespace rtac

