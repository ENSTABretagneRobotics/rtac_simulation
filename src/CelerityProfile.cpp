#include <rtac_simulation/CelerityProfile.h>

#include <rtac_base/containers/HostVector.h>
#include <rtac_base/interpolation.h>

namespace rtac { namespace simulation {

/**
 * This interpolates celerityData with a cubic spline to generate a uniformly
 * sampled profile.
 *
 * The gradient of the celerity profile is generate using the derivative of the
 * computed splines.
 */
CelerityProfile::CelerityProfile(unsigned int inputSize,
                                 const float* heightData,
                                 const float* celerityData,
                                 unsigned int profileSize) :
    zBounds_(heightData[0], heightData[inputSize-1])
{
    using Interpolator = rtac::algorithm::InterpolatorCubicSpline<float>;

    Interpolator interp(VectorView<const float>(inputSize, heightData),
                        VectorView<const float>(inputSize, celerityData));
    
    auto h = HostVector<float>::linspace(heightData[0], heightData[inputSize-1],
                                         profileSize);
    HostVector<float> celerity(h.size());
    HostVector<float> gradient(h.size());
    interp.interpolate(h, celerity, gradient);

    //precomputation to save time when running simulation
    for(unsigned int i = 0; i < gradient.size(); i++) {
        float c = celerity[i];
        if(fabs(c) < 1.0e-6) {
            gradient[i] = 0;
        }
        else {
            gradient[i] *= -1.0 / (c*c);
        }
    }

    celerity_.set_image(celerity.size(), 1, celerity.data());
    gradient_.set_image(gradient.size(), 1, gradient.data());

    // setting linear interpolation for celerity and gradient (and clamp to
    // edge border mode)
    celerity_.set_filter_mode(cuda::Texture2D<float>::FilterLinear, false);
    celerity_.set_wrap_mode(  cuda::Texture2D<float>::WrapClamp,    true);
    gradient_.set_filter_mode(cuda::Texture2D<float>::FilterLinear, false);
    gradient_.set_wrap_mode(  cuda::Texture2D<float>::WrapClamp,    true);
}

CelerityProfile::Ptr CelerityProfile::Create(unsigned int inputSize,
                                             const float* heightData,
                                             const float* celerityData,
                                             unsigned int profileSize)
{
    if(heightData[0] > heightData[inputSize-1]) {
        throw std::runtime_error("Should be in ascending order");
    }
    return Ptr(new CelerityProfile(inputSize, heightData, celerityData, profileSize));
}

CelerityProfileView CelerityProfile::view() const
{
    CelerityProfileView view;

    view.a_ = 1.0f / zBounds_.length();
    view.b_ = - zBounds_.lower * view.a_;
    view.celerity_ = celerity_.texture();
    view.gradient_ = gradient_.texture();

    return view;
}

} //namespace simulation
} //namespace rtac
