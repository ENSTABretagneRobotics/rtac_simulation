#ifndef _DEF_RTAC_SIMULATION_WAVEFORM_H_
#define _DEF_RTAC_SIMULATION_WAVEFORM_H_

#include <memory>
#include <iostream>

#include <rtac_base/types/Complex.h>
#include <rtac_base/signal_helpers.h>
#include <rtac_base/containers/HostVector.h>
#include <rtac_base/cuda/CudaVector.h>

namespace rtac { namespace simulation {

class Waveform;

/**
 * Iterator on Waveform value (no no-const iterator available).
 */
class WaveformIterator
{
    protected:

    const Waveform* waveform_;
    unsigned int    index_;
    
    public:
    
    WaveformIterator() = default;
    WaveformIterator& operator=(const WaveformIterator&) = default;

    WaveformIterator(const Waveform* waveform, unsigned int index) :
        waveform_(waveform),
        index_(index)
    {}

    WaveformIterator& operator++() { index_++; return *this; }
    WaveformIterator  operator++(int) const {
        WaveformIterator res(*this);
        (*this)++;
        return res;
    }

    bool operator==(const WaveformIterator& other) const {
        return waveform_ == other.waveform_ && index_ == other.index_;
    }
    bool operator!=(const WaveformIterator& other) const {
        return !(*this == other);
    }

    //Complex<float> operator*() const { return (*waveform_)[index_]; }
    //Complex<float> operator*() const { return waveform_->operator[](index_); }
    Complex<float> operator*() const;
};

/**
 * This class represents the waveform output from the emitting hydrophone.
 * 
 * It gathers informations such as frequency and duration (really ?)
 *
 * The complex output is the hilbert transform of the real waveform (computed
 * with the fourier transform of the input). This is necessary because of the
 * way the simulation propagates phase information.
 *
 * This is an abstract base type and should be reimplemented in a subclass.
 */
class Waveform
{
    public:

    using Ptr      = std::shared_ptr<Waveform>;
    using ConstPtr = std::shared_ptr<const Waveform>;

    protected:

    bool fixed_;
    Waveform(bool fixed) : fixed_(fixed) {}

    public:

    WaveformIterator begin() const { return WaveformIterator(this, 0);            }
    WaveformIterator end()   const { return WaveformIterator(this, this->size()); }
    bool fixed() const { return fixed_; }

    /**
     * This method should trigger a reconfiguration of the Waveform
     */
    virtual void set_duration(float duration) = 0;
    virtual float duration()    const = 0;
    virtual unsigned int size() const = 0;
    virtual Complex<float> operator[](unsigned int idx) const = 0;
    virtual Waveform::Ptr copy() const = 0;
};

inline Complex<float> WaveformIterator::operator*() const { return (*waveform_)[index_]; }

class Waveform_Sine : public Waveform
{
    public:

    using Ptr      = std::shared_ptr<Waveform>;
    using ConstPtr = std::shared_ptr<const Waveform>;

    protected:

    float frequency_;
    float duration_;
    unsigned int oversampling_;

    Waveform_Sine(float frequency, float duration, bool fixed,
                  unsigned int oversampling) :
        Waveform(fixed),
        frequency_(frequency),
        duration_(duration),
        oversampling_(oversampling)
    {}

    public:

    static Ptr Create(float frequency, float duration,
                      bool fixed, unsigned int oversampling = 8)
    {
        return Ptr(new Waveform_Sine(frequency, duration, fixed, oversampling));
    }

    void set_duration(float duration) { if(!this->fixed()) duration_ = duration; }
    
    float sample_rate() const { return 2*frequency_*oversampling_; }
    float frequency()   const { return frequency_; }
    float duration()    const { return duration_;  }
    unsigned int size() const { return this->sample_rate() * duration_; }
    Complex<float> operator[](unsigned int idx) const {
        float phase = idx*(2.0*M_PI*frequency_ / this->sample_rate());
        return Complex<float>(std::cos(phase), std::sin(phase));
    }
    virtual Waveform_Sine::Ptr copy() const {
        return Waveform_Sine::Create(frequency_, duration_, this->fixed(), oversampling_);
    }
};

} //namespace simulation
} //namespace rtac

inline std::ostream& operator<<(std::ostream& os, const rtac::simulation::Waveform& waveform)
{
    os << "Waveform (duration : " << waveform.duration()
       << ", size : " << waveform.size() << ") :";
    for(auto v : waveform) {
        os << ' ' << v;
    }
    return os;
}

#endif //_DEF_RTAC_SIMULATION_WAVEFORM_H_
