#include <rtac_simulation/Directivity.h>

#include <rtac_base/signal_helpers.h>

namespace rtac { namespace simulation {

Directivity::Directivity(const HostImage& data)
{
    this->load_default_configuration();
    texture_.set_image(data.width(), data.height(), data.data());
}

Directivity::Directivity(const DeviceImage& data)
{
    this->load_default_configuration();
    texture_.set_image(data.width(), data.height(), data.container());
}

void Directivity::load_default_configuration()
{
    texture_.set_filter_mode(rtac::cuda::Texture2D<float>::FilterLinear, false);
    texture_.set_wrap_mode(rtac::cuda::Texture2D<float>::WrapMirror, true);
}

DirectivityView Directivity::view() const
{
    return DirectivityView({texture_.texture(), 1.0f / M_PI, 1.0f / M_PI});
}

Directivity::Ptr Directivity::from_sinc_parameters(float bearingAperture, 
                                                   float elevationAperture,
                                                   unsigned int oversampling)
{
    signal::SincFunction<float> bearingDirectivity(
        0.0f, 2.0*1.39156*M_PI / bearingAperture, oversampling);
    signal::SincFunction<float> elevationDirectivity(
        0.0f, 2.0*1.39156*M_PI / elevationAperture, oversampling);
    
    HostImage data({(unsigned int)bearingDirectivity.size(), 
                    (unsigned int)elevationDirectivity.size()}); 
    for(int h = 0; h < data.height(); h++) {
        for(int w = 0; w < data.width(); w++) {
            data(h,w) = bearingDirectivity.function()[w]
                      * elevationDirectivity.function()[h];
        }
    }

    return Directivity::Create(data);
}

} //namespace simulation
} //namespace rtac
