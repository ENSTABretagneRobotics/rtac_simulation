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

/**
 * This is bad. should have revolution symmetry
 */
Directivity::Ptr Directivity::from_sinc_parameters(float bearingAperture, 
                                                   float elevationAperture,
                                                   unsigned int oversampling)
{
    bearingAperture   *= M_PIf / 180.0f;
    elevationAperture *= M_PIf / 180.0f;

    signal::SincFunction<float> bearingDirectivity(
        0.0f, 2.0*1.39156*M_PI / bearingAperture, oversampling);
    signal::SincFunction<float> elevationDirectivity(
        0.0f, 2.0*1.39156*M_PI / elevationAperture, oversampling);
    
    HostImage data((unsigned int)bearingDirectivity.size(), 
                   (unsigned int)elevationDirectivity.size()); 
    for(int h = 0; h < data.height(); h++) {
        for(int w = 0; w < data.width(); w++) {
            data(h,w) = bearingDirectivity.function()[w]
                      * elevationDirectivity.function()[h];
        }
    }

    return Directivity::Create(data);
}

Directivity::Ptr Directivity::rectangle_antenna(float width, float height, float wavelength,
                                                const std::string& baffleMode,
                                                unsigned int oversampling)
{
    unsigned int Nw = 2*(unsigned int)(0.5f*oversampling*2*M_PI*width  / wavelength + 1);
    unsigned int Nh = 2*(unsigned int)(0.5f*oversampling*2*M_PI*height / wavelength + 1);

    HostImage data(Nw, Nh);
    for(unsigned int h = 0; h < data.height(); h++) {
        double elevation = M_PI * h / data.height();
        float eValue = std::sin(M_PI*height/wavelength*std::sin(elevation))
                     / (M_PI*height/wavelength*std::sin(elevation));
        if(h == 0) eValue = 1;
        for(unsigned int w = 0; w < data.width(); w++) {
            double bearing = M_PI * w / data.width();
            float bValue = std::sin(M_PI*width/wavelength*std::sin(bearing))
                         / (M_PI*width/wavelength*std::sin(bearing));
            if(w == 0) bValue = 1;

            data(h,w)  = (eValue * bValue) * (eValue  * bValue);
        }
    }

    if(baffleMode == "single-sided") {
        for(unsigned int h = data.height() / 2; h < data.height(); h++) {
            for(unsigned int w = 0; w < data.width(); w++) {
                data(h,w) = 0;
            }
        }
    }
    else if(baffleMode == "cardioid") {
        for(unsigned int h = 0; h < data.height(); h++) {
            double elevation = M_PI * h / data.height();
            for(unsigned int w = 0; w < data.width(); w++) {
                data(h,w) *= 0.5f*(1.0f + cos(elevation));
            }
        }
    }

    return Directivity::Create(data);
}


} //namespace simulation
} //namespace rtac
