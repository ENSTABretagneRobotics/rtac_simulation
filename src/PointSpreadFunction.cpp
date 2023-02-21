#include <rtac_simulation/PointSpreadFunction.h>

namespace rtac { namespace simulation {

RangePSF_Sine::RangePSF_Sine(float soundCelerity, float frequency,
                             float pulseLength, unsigned int oversampling) :
    RangePSF(pulseLength)
{
    this->reconfigure(soundCelerity, frequency, pulseLength, oversampling);
}

void RangePSF_Sine::reconfigure(float soundCelerity, float frequency,
                                float pulseLength, unsigned int oversampling)
{
    soundCelerity_ = soundCelerity;
    frequency_     = frequency;
    wavelength_    = soundCelerity_ / frequency_;
    oversampling_  = oversampling;
    this->pulseLength_    = pulseLength;

    function_ = std::make_shared<signal::SineFunction<float>>(
        this->pulse_length() / wavelength_, oversampling_);
}

/**
 * This reconfigure the PSF pulseLength while keeping sound celerity, frequency and
 * wavelength set.
 */
void RangePSF_Sine::reconfigure(float pulseLength)
{
    this->pulseLength_ = pulseLength;
    function_ = std::make_shared<signal::SineFunction<float>>(
        this->pulse_length() / wavelength_, oversampling_);
}

RangePSF_ComplexSine::RangePSF_ComplexSine(float soundCelerity, float frequency,
                             float pulseLength, unsigned int oversampling) :
    RangePSF(pulseLength)
{
    this->reconfigure(soundCelerity, frequency, pulseLength, oversampling);
}

void RangePSF_ComplexSine::reconfigure(float soundCelerity, float frequency,
                                float pulseLength, unsigned int oversampling)
{
    soundCelerity_ = soundCelerity;
    frequency_     = frequency;
    wavelength_    = soundCelerity_ / frequency_;
    oversampling_  = oversampling;
    this->pulseLength_    = pulseLength;

    function_ = std::make_shared<signal::ComplexSineFunction<float>>(
        this->pulse_length() / wavelength_, oversampling_);
}

/**
 * This reconfigure the PSF pulseLength while keeping sound celerity, frequency and
 * wavelength set.
 */
void RangePSF_ComplexSine::reconfigure(float pulseLength)
{
    this->pulseLength_ = pulseLength;
    function_ = std::make_shared<signal::ComplexSineFunction<float>>(
        this->pulse_length() / wavelength_, oversampling_);
}

} //namespace simulation
} //namespace rtac


