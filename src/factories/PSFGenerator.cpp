#include <rtac_simulation/factories/PSFGenerator.h>

namespace rtac { namespace simulation {

RangePSF_Sine::RangePSF_Sine(float pulseLength, float soundCelerity,
                             float frequency, unsigned int oversampling) :
    RangePSF(pulseLength)
{
    this->set_pulse_length(pulseLength, soundCelerity, frequency, oversampling);
}

void RangePSF_Sine::set_pulse_length(float pulseLength, float soundCelerity,
                                     float frequency, unsigned int oversampling)
{
    soundCelerity_     = soundCelerity;
    frequency_         = frequency;
    wavelength_        = soundCelerity_ / frequency_;
    oversampling_      = oversampling;
    this->pulseLength_ = pulseLength;

    function_ = std::make_shared<signal::SineFunction<float>>(
        this->pulse_length() / wavelength_, oversampling_);
}

/**
 * This set_pulse_length the PSF pulseLength while keeping sound celerity, frequency and
 * wavelength set.
 */
void RangePSF_Sine::set_pulse_length(float pulseLength)
{
    this->pulseLength_ = pulseLength;
    function_ = std::make_shared<signal::SineFunction<float>>(
        this->pulse_length() / wavelength_, oversampling_);
}

RangePSF_ComplexSine::RangePSF_ComplexSine(float pulseLength,
                                           float soundCelerity, 
                                           float frequency,
                                           unsigned int oversampling) :
    RangePSF(pulseLength)
{
    this->set_pulse_length(pulseLength, soundCelerity, frequency, oversampling);
}

void RangePSF_ComplexSine::set_pulse_length(float pulseLength,
                                            float soundCelerity,
                                            float frequency,
                                            unsigned int oversampling)
{
    soundCelerity_     = soundCelerity;
    frequency_         = frequency;
    wavelength_        = soundCelerity_ / frequency_;
    oversampling_      = oversampling;
    this->pulseLength_ = pulseLength;

    function_ = std::make_shared<signal::ComplexSineFunction<float>>(
        this->pulse_length() / wavelength_, oversampling_);
}

/**
 * This set_pulse_length the PSF pulseLength while keeping sound celerity, frequency and
 * wavelength set.
 */
void RangePSF_ComplexSine::set_pulse_length(float pulseLength)
{
    this->pulseLength_ = pulseLength;
    function_ = std::make_shared<signal::ComplexSineFunction<float>>(
        this->pulse_length() / wavelength_, oversampling_);
}

} //namespace simulation
} //namespace rtac


