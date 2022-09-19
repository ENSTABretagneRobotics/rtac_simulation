#include <rtac_simulation/examples/blueprint_oculus.h>

#include <rtac_simulation/helpers/receiver_factories.h>

namespace rtac { namespace simulation { namespace oculus {

Directivity::ConstPtr OculusReceiverLF::make_directivity()
{
    return Directivity::from_sinc_parameters(130.0f, 20.0f);
}

OculusReceiverLF::Kernel::ConstPtr OculusReceiverLF::make_kernel(float rangeResolution)
{
    float bearingResolution = 0.6f;
    float bearingSpan       = 130.0f;
    float pulseLength       = 2*rangeResolution;
    float waveLength        = 0.0012178;

    return simple_polar_kernel<float>(bearingResolution, bearingSpan,
                                      pulseLength, waveLength);
}

OculusReceiverLF::OculusReceiverLF() :
    PolarReceiver2D<float>(make_directivity(), nullptr, nullptr)
{}

void OculusReceiverLF::update_target(float maxRange, unsigned int nRanges,
                                     const std::vector<float>& bearings)
{
    std::vector<float> ranges(nRanges);
    for(int n = 0; n < nRanges; n++) {
        ranges[n] = (maxRange * n) / (nRanges - 1);
    }
    if(!this->target_) {
        this->target_ = PolarTarget2D<Complex<float>>::Create(bearings, ranges);
    }
    else {
        this->target_->update(bearings, ranges);
    }

    this->psf_ = make_kernel(this->target()->range_resolution());
}

} //namespace examples
} //namespace simulation
} //namespace rtac

