#include <rtac_simulation/PointSpreadFunction.h>

namespace rtac { namespace simulation {

RangePSF_Sin::RangePSF_Sin(float soundCelerity, float frequency,
                           float pulseLength, unsigned int oversampling) :
    RangePSF(pulseLength)
{
    this->reconfigure(soundCelerity, frequency, pulseLength, oversampling);
}

void RangePSF_Sin::reconfigure(float soundCelerity, float frequency,
                               float pulseLength, unsigned int oversampling)
{
    soundCelerity_ = soundCelerity;
    frequency_     = frequency;
    wavelength_    = soundCelerity_ / frequency_;
    oversampling_  = oversampling;
    this->pulseLength_    = pulseLength;

    function_ = std::make_shared<signal::SinFunction<float>>(
        this->pulse_length() / wavelength_, oversampling_);
}

/**
 * This reconfigure the PSF pulseLength while keeping sound celerity, frequency and
 * wavelength set.
 */
void RangePSF_Sin::reconfigure(float pulseLength)
{
    this->pulseLength_ = pulseLength;
    function_ = std::make_shared<signal::SinFunction<float>>(
        this->pulse_length() / wavelength_, oversampling_);
}

} //namespace simulation
} //namespace rtac


