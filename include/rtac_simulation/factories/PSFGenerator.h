#ifndef _DEF_RTAC_SIMULATION_POINT_SPREAD_FUNCTION_H_
#define _DEF_RTAC_SIMULATION_POINT_SPREAD_FUNCTION_H_

#include <memory>

#include <rtac_base/types/Complex.h>
#include <rtac_base/signal_helpers.h>

namespace rtac { namespace simulation {

class PSFGenerator// : public std::enable_shared_from_this<PSFGenerator>
{
    public:
    
    using Ptr      = std::shared_ptr<PSFGenerator>;
    using ConstPtr = std::shared_ptr<const PSFGenerator>;

    protected:

    //PSFGenerator(float span) : span_(span) {}
    PSFGenerator() = default;

    public:

    virtual float span()        const = 0;
    virtual unsigned int size() const = 0;
    virtual bool is_complex()   const = 0;
};

struct PSFGenerator_Real : public PSFGenerator
{
    using Ptr      = std::shared_ptr<PSFGenerator_Real>;
    using ConstPtr = std::shared_ptr<const PSFGenerator_Real>;

    bool is_complex() const { return false; }

    virtual float operator[](unsigned int idx) const = 0;
};

struct PSFGenerator_Complex : public PSFGenerator
{
    using Ptr      = std::shared_ptr<PSFGenerator_Complex>;
    using ConstPtr = std::shared_ptr<const PSFGenerator_Complex>;

    bool is_complex() const { return true; }

    virtual Complex<float> operator[](unsigned int idx) const = 0;
};

class RangePSF
{
    public:
    
    using Ptr      = std::shared_ptr<RangePSF>;
    using ConstPtr = std::shared_ptr<const RangePSF>;

    protected:
   
    float pulseLength_;

    RangePSF(float pulseLength) : pulseLength_(pulseLength) {}

    public:

    float pulse_length() const { return pulseLength_; }

    virtual void reconfigure(float pulseLength) { pulseLength_ = pulseLength; }
};

class RangePSF_Square : public RangePSF, public PSFGenerator_Real
{
    public:

    using Ptr      = std::shared_ptr<RangePSF_Square>;
    using ConstPtr = std::shared_ptr<const RangePSF_Square>;

    protected:

    RangePSF_Square(float pulseLength) : RangePSF(pulseLength) {}

    public:

    static Ptr Create(float pulseLength) { return Ptr(new RangePSF_Square(pulseLength)); }

    unsigned int size() const { return 1; }
    float span() const { return this->pulse_length(); }
    float operator[](unsigned int idx) const { return 1.0f; }
};

class RangePSF_Sine : public RangePSF, public PSFGenerator_Real
{
    public:

    using Ptr      = std::shared_ptr<RangePSF_Sine>;
    using ConstPtr = std::shared_ptr<const RangePSF_Sine>;

    protected:

    float soundCelerity_;
    float frequency_;
    float wavelength_;
    unsigned int oversampling_;
    std::shared_ptr<signal::SineFunction<float>> function_;

    RangePSF_Sine(float soundCelerity, float frequency,
                  float pulseLength, unsigned int oversampling = 8);
;
    public:

    static Ptr Create(float soundCelerity, float frequency,
                      float pulseLength, unsigned int oversampling = 8) 
    {
        return Ptr(new RangePSF_Sine(soundCelerity, frequency, pulseLength, oversampling));
    }

    void reconfigure(float soundCelerity, float frequency,
                     float pulseLength, unsigned int oversampling = 8);
    void reconfigure(float pulseLength);

    float span() const { return this->pulse_length(); }
    unsigned int size() const { return function_->size(); }
    float operator[](unsigned int idx) const { return function_->function()[idx]; }
};

class RangePSF_ComplexSine : public RangePSF, public PSFGenerator_Complex
{
    public:

    using Ptr      = std::shared_ptr<RangePSF_ComplexSine>;
    using ConstPtr = std::shared_ptr<const RangePSF_ComplexSine>;

    protected:

    float soundCelerity_;
    float frequency_;
    float wavelength_;
    unsigned int oversampling_;
    std::shared_ptr<signal::ComplexSineFunction<float>> function_;

    RangePSF_ComplexSine(float soundCelerity, float frequency,
                         float pulseLength, unsigned int oversampling = 8);

    public:

    static Ptr Create(float soundCelerity, float frequency,
                      float pulseLength, unsigned int oversampling = 8) 
    {
        return Ptr(new RangePSF_ComplexSine(soundCelerity, frequency,
                                            pulseLength, oversampling));
    }

    void reconfigure(float soundCelerity, float frequency,
                     float pulseLength, unsigned int oversampling = 8);
    void reconfigure(float pulseLength);

    float span() const { return this->pulse_length(); }
    unsigned int size() const { return function_->size(); }
    Complex<float> operator[](unsigned int idx) const { return function_->function()[idx]; }
};


class BearingPSF_Real : public PSFGenerator_Real
{
    public:

    using Ptr      = std::shared_ptr<BearingPSF_Real>;
    using ConstPtr = std::shared_ptr<const BearingPSF_Real>;

    protected:

    std::vector<float> coeffs_;

    BearingPSF_Real(const std::vector<float>& coeffs) : coeffs_(coeffs) {}

    public:

    static Ptr Create(const std::vector<float>& coeffs) {
        return Ptr(new BearingPSF_Real(coeffs));
    }

    float span() const { return coeffs_.back() - coeffs_.front(); }
    unsigned int size() const { return coeffs_.size(); }
    float operator[](unsigned int idx) const { return coeffs_[idx]; }

};

class BearingPSF_Sinc : public BearingPSF_Real
{
    public:

    using Ptr      = std::shared_ptr<BearingPSF_Sinc>;
    using ConstPtr = std::shared_ptr<const BearingPSF_Sinc>;

    protected:

    BearingPSF_Sinc(float span, float resolution, unsigned int oversampling) : 
        BearingPSF_Real(signal::SincFunction(span / resolution, oversampling).function())
    {}

    public:

    static Ptr Create(float span, float resolution, unsigned int oversampling = 8) {
        return Ptr(new BearingPSF_Sinc(span, resolution, oversampling));
    }
};

class BearingPSF_Gauss : public BearingPSF_Real
{
    public:

    using Ptr      = std::shared_ptr<BearingPSF_Gauss>;
    using ConstPtr = std::shared_ptr<const BearingPSF_Gauss>;

    protected:

    BearingPSF_Gauss(float span, float sigma, unsigned int oversampling) : 
        BearingPSF_Real(signal::GaussFunction(span, sigma, oversampling).function())
    {}

    public:

    static Ptr Create(float span, float sigma, unsigned int oversampling = 8) {
        return Ptr(new BearingPSF_Gauss(span, sigma, oversampling));
    }
};

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_POINT_SPREAD_FUNCTION_H_
