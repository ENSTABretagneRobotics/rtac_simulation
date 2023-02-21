#include <rtac_simulation/PointSpreadFunction.h>

#include <rtac_base/cuda/texture_utils.h>

namespace rtac { namespace simulation {

void PointSpreadFunction2D::set_pulse_length(float pulseLength)
{
    auto ptr = std::dynamic_pointer_cast<RangePSF>(rangePSF_);
    if(!ptr) {
        throw std::runtime_error(
            "PointSpreadFunction : rangePSF must be derived from the RangePSF type");
    }
    ptr->set_pulse_length(pulseLength);
    this->generate_kernel();
}

std::shared_ptr<PSF2D_Real> PointSpreadFunction2D::real_cast()
{
    return std::dynamic_pointer_cast<PSF2D_Real>(this->shared_from_this());
}

std::shared_ptr<PSF2D_Complex> PointSpreadFunction2D::complex_cast()
{
    return std::dynamic_pointer_cast<PSF2D_Complex>(this->shared_from_this());
}

std::shared_ptr<const PSF2D_Real> PointSpreadFunction2D::real_cast() const
{
    return std::dynamic_pointer_cast<const PSF2D_Real>(this->shared_from_this());
}

std::shared_ptr<const PSF2D_Complex> PointSpreadFunction2D::complex_cast() const
{
    return std::dynamic_pointer_cast<const PSF2D_Complex>(this->shared_from_this());
}

void PSF2D_Real::generate_kernel()
{
    auto bearingPtr = std::dynamic_pointer_cast<PSFGenerator_Real>(this->bearingPSF_);
    auto rangePtr   = std::dynamic_pointer_cast<PSFGenerator_Real>(this->rangePSF_);

    if(!bearingPtr || !rangePtr) {
        throw std::runtime_error("Invalid PSFGenerators for PSF2D_Real");
    }

    const auto& bearings = *bearingPtr;
    const auto& ranges   = *rangePtr;

    Image<float> data(bearings.size(), ranges.size());   
    for(unsigned int h = 0; h < data.height(); h++) {
        for(unsigned int w = 0; w < data.width(); w++) {
            data(h,w) = bearings[w] * ranges[h];
        }
    }
    
    data_.set_image(data.width(), data.height(), data.data());
}

Image<float, cuda::DeviceVector> PSF2D_Real::render() const
{
    Image<float, cuda::DeviceVector> res(this->width(), this->height());
    render_texture(data_, res.view());
    return res;
}

template <typename BearingPSF_T, typename RangePSF_T>
void fill_kernel_data(cuda::Texture2D<float2>& out,
                      const std::shared_ptr<BearingPSF_T>& bearingPtr,
                      const std::shared_ptr<RangePSF_T>&   rangePtr)
{
    if(!bearingPtr || !rangePtr) {
        throw std::runtime_error("PSF2D_Complex error : invalid PSFGenerator.");
    }
    const auto& bearings = *bearingPtr;
    const auto& ranges   = *rangePtr;

    Image<Complex<float>> data(bearings.size(), ranges.size());   
    for(unsigned int h = 0; h < data.height(); h++) {
        for(unsigned int w = 0; w < data.width(); w++) {
            data(h,w) = bearings[w] * ranges[h];
        }
    }

    out.set_image(data.width(), data.height(), (const float2*)data.data());
}

void PSF2D_Complex::generate_kernel()
{
    if(this->bearingPSF_->is_complex()) {
        auto bearingPtr = std::dynamic_pointer_cast<PSFGenerator_Complex>(this->bearingPSF_);
        if(this->rangePSF_->is_complex()) {
            auto rangePtr = std::dynamic_pointer_cast<PSFGenerator_Complex>(this->rangePSF_);
            fill_kernel_data(data_, bearingPtr, rangePtr);
        }
        else {
            auto rangePtr = std::dynamic_pointer_cast<PSFGenerator_Real>(this->rangePSF_);
            fill_kernel_data(data_, bearingPtr, rangePtr);
        }
    }
    else {
        auto bearingPtr = std::dynamic_pointer_cast<PSFGenerator_Real>(this->bearingPSF_);
        if(this->rangePSF_->is_complex()) {
            auto rangePtr = std::dynamic_pointer_cast<PSFGenerator_Complex>(this->rangePSF_);
            fill_kernel_data(data_, bearingPtr, rangePtr);
        }
        else {
            auto rangePtr = std::dynamic_pointer_cast<PSFGenerator_Real>(this->rangePSF_);
            fill_kernel_data(data_, bearingPtr, rangePtr);
        }
    }
}

Image<float2, cuda::DeviceVector> PSF2D_Complex::render() const
{
    Image<float2, cuda::DeviceVector> res(this->width(), this->height());
    render_texture(data_, res.view());
    return res;
}

PointSpreadFunction2D::Ptr make_point_spread_function(const PSFGenerator::Ptr& bearingPSF,
                                                      const PSFGenerator::Ptr& rangePSF)
{
    if(bearingPSF->is_complex() || rangePSF->is_complex()) {
        return PSF2D_Complex::Create(bearingPSF, rangePSF);
    }
    else {
        return PSF2D_Real::Create(bearingPSF, rangePSF);
    }
}
 

} //namespace simulation
} //namespace rtac

std::ostream& operator<<(std::ostream& os, const rtac::simulation::PointSpreadFunction2D& psf)
{
    if(psf.is_complex()) {
        os << "PointSpreadFunction(Complex";
    }
    else {
        os << "PointSpreadFunction(Real";
    }
    os << ", " << psf.bearing_size() << 'x' << psf.range_size()
       << ", " << psf.bearing_span() << 'x' << psf.range_span() << ')';
    return os;
}


