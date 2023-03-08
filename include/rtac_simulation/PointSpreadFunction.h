#ifndef _DEF_RTAC_SIMULATION_POINT_SPREAD_FUNCTION_H_
#define _DEF_RTAC_SIMULATION_POINT_SPREAD_FUNCTION_H_

#include <memory>

#include <rtac_base/types/Complex.h>
#include <rtac_base/containers/Image.h>
#include <rtac_base/cuda/DeviceVector.h>
#include <rtac_base/cuda/Texture2D.h>

#include <rtac_simulation/Waveform.h>
#include <rtac_simulation/BeamDirectivity.h>
#include <rtac_simulation/factories/PSFGenerator.h>
#include <rtac_simulation/ReductionKernel.h>

namespace rtac { namespace simulation {

class PointSpreadFunction2D_2
{
    public:

    using Ptr      = std::shared_ptr<PointSpreadFunction2D_2>;
    using ConstPtr = std::shared_ptr<const PointSpreadFunction2D_2>;

    protected:
    
    Waveform::Ptr           waveform_;
    BeamDirectivity::Ptr    beamDirectivity_;
    cuda::Texture2D<float2> data_;

    PointSpreadFunction2D_2(const Waveform::Ptr& waveform,
                            const BeamDirectivity::Ptr& beamDirectivity) :
        waveform_(waveform),
        beamDirectivity_(beamDirectivity)
    {
        this->generate_data();
    }
    void generate_data();

    public:
    
    static Ptr Create(const Waveform::Ptr& waveform,
                      const BeamDirectivity::Ptr& beamDirectivity)
    {
        return Ptr(new PointSpreadFunction2D_2(waveform, beamDirectivity));
    }

    float bearing_span() const { return beamDirectivity_->span(); }
    float range_span(float scaling = 1.0f) const { return scaling*waveform_->duration(); }
    unsigned int bearing_size() const { return beamDirectivity_->size(); }
    unsigned int range_size()   const { return waveform_->size();        }
    unsigned int width()        const { return this->bearing_size();     }
    unsigned int height()       const { return this->range_size();       }

    Waveform::ConstPtr        waveform()         const { return waveform_; }
    BeamDirectivity::ConstPtr beam_directivity() const { return beamDirectivity_; }

    void set_pulse_duration(float duration) {
        waveform_->set_duration(duration);
        this->generate_data();
    }

    KernelView2D<Complex<float>> kernel(float rangeScaling = 1.0f) const
    {
        KernelView2D<Complex<float>> kernel;
        kernel.xScaling_ = float2{1.0f / this->bearing_span(), 0.5f};
        kernel.yScaling_ = float2{1.0f / this->range_span(rangeScaling), 0.5f};
        kernel.function_ = data_.texture();
        return kernel;
    }
};

class PSF2D_Real;
class PSF2D_Complex;

class PointSpreadFunction2D : public std::enable_shared_from_this<PointSpreadFunction2D>
{
    public:

    using Ptr      = std::shared_ptr<PointSpreadFunction2D>;
    using ConstPtr = std::shared_ptr<const PointSpreadFunction2D>;

    protected:

    PSFGenerator::Ptr bearingPSF_;
    PSFGenerator::Ptr rangePSF_;

    PointSpreadFunction2D(const PSFGenerator::Ptr& bearingPSF,
                          const PSFGenerator::Ptr& rangePSF) :
        bearingPSF_(bearingPSF),
        rangePSF_(rangePSF)
    {
        if(!std::dynamic_pointer_cast<RangePSF>(rangePSF)) {
            throw std::runtime_error(
                "PointSpreadFunction : rangePSF must be derived from the RangePSF type");
        }
    }

    virtual void generate_kernel() = 0;
    
    public:

    float bearing_span()        const { return bearingPSF_->span(); }
    float range_span()          const { return rangePSF_->span();   }
    unsigned int bearing_size() const { return bearingPSF_->size(); }
    unsigned int range_size()   const { return rangePSF_->size();   }
    unsigned int width()        const { return bearingPSF_->size(); }
    unsigned int height()       const { return rangePSF_->size();   }
    bool is_complex() const { 
        return bearingPSF_->is_complex() || rangePSF_->is_complex();
    }

    void set_pulse_length(float pulseLength);
    
    std::shared_ptr<PSF2D_Real>          real_cast();
    std::shared_ptr<PSF2D_Complex>       complex_cast();
    std::shared_ptr<const PSF2D_Real>    real_cast()    const;
    std::shared_ptr<const PSF2D_Complex> complex_cast() const;
};

class PSF2D_Real : public PointSpreadFunction2D
{
    public:

    using Ptr      = std::shared_ptr<PSF2D_Real>;
    using ConstPtr = std::shared_ptr<const PSF2D_Real>;

    protected:

    cuda::Texture2D<float> data_; 

    PSF2D_Real(const PSFGenerator::Ptr& bearingPSF,
               const PSFGenerator::Ptr& rangePSF) :
        PointSpreadFunction2D(bearingPSF, rangePSF)
    {
        data_.set_filter_mode(cuda::Texture2D<float>::FilterLinear, false);
        data_.set_wrap_mode(cuda::Texture2D<float>::WrapBorder, true);
        this->PSF2D_Real::generate_kernel();
    }

    void generate_kernel();
    
    public:

    static Ptr Create(const PSFGenerator::Ptr& bearingPSF,
                      const PSFGenerator::Ptr& rangePSF)
    {
        return Ptr(new PSF2D_Real(bearingPSF, rangePSF));
    }

    const cuda::Texture2D<float>& texture() const { return data_; }

    KernelView2D<float> kernel() const {
        KernelView2D<float> kernel;
        kernel.xScaling_ = float2{1.0f / this->bearing_span(), 0.5f};
        kernel.yScaling_ = float2{1.0f / this->range_span(), 0.5f};
        kernel.function_ = data_.texture();
        return kernel;
    }

    Image<float, cuda::DeviceVector> render() const;
};

class PSF2D_Complex : public PointSpreadFunction2D
{
    public:

    using Ptr      = std::shared_ptr<PSF2D_Complex>;
    using ConstPtr = std::shared_ptr<const PSF2D_Complex>;

    protected:

    cuda::Texture2D<float2> data_; 

    PSF2D_Complex(const PSFGenerator::Ptr& bearingPSF,
                  const PSFGenerator::Ptr& rangePSF) :
        PointSpreadFunction2D(bearingPSF, rangePSF)
    {
        data_.set_filter_mode(cuda::Texture2D<float2>::FilterLinear, false);
        data_.set_wrap_mode(cuda::Texture2D<float2>::WrapBorder, true);
        this->PSF2D_Complex::generate_kernel();
    }

    void generate_kernel();
    
    public:

    static Ptr Create(const PSFGenerator::Ptr& bearingPSF,
                      const PSFGenerator::Ptr& rangePSF)
    {
        return Ptr(new PSF2D_Complex(bearingPSF, rangePSF));
    }

    const cuda::Texture2D<float2>& texture() const { return data_; }

    KernelView2D<Complex<float>> kernel() const {
        KernelView2D<Complex<float>> kernel;
        kernel.xScaling_ = float2{1.0f / this->bearing_span(), 0.5f};
        kernel.yScaling_ = float2{1.0f / this->range_span(),   0.5f};
        kernel.function_ = data_.texture();
        return kernel;
    }

    Image<float2, cuda::DeviceVector> render() const;
};

PointSpreadFunction2D::Ptr make_point_spread_function(const PSFGenerator::Ptr& bearingPSF,
                                                      const PSFGenerator::Ptr& rangePSF);

PSF2D_Real::Ptr         real_cast(const PointSpreadFunction2D::Ptr& ptr);
PSF2D_Real::ConstPtr    real_cast(const PointSpreadFunction2D::ConstPtr& ConstPtr);
PSF2D_Complex::Ptr      complex_cast(const PointSpreadFunction2D::Ptr& ptr);
PSF2D_Complex::ConstPtr complex_cast(const PointSpreadFunction2D::ConstPtr& ConstPtr);

} //namespace simulation
} //namespace rtac

std::ostream& operator<<(std::ostream& os, const rtac::simulation::PointSpreadFunction2D& psf);

#endif //_DEF_RTAC_SIMULATION_POINT_SPREAD_FUNCTION_H_
