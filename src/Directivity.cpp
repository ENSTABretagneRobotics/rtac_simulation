#include <rtac_simulation/Directivity.h>

#include <rtac_base/signal_helpers.h>

#include <math.h>

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
        for(unsigned int h = data.height(); h < data.height(); h++) {
            for(unsigned int w = 0; w < data.width(); w++) {
                if(h > data.height() / 2 || w > data.width() / 2)
                    data(h,w) = 0;
            }
        }
    }
    else if(baffleMode == "cardioid") {
        throw std::runtime_error("baffleMode 'cardioid' unsupported");
        //for(unsigned int h = 0; h < data.height(); h++) {
        //    double elevation = M_PI * h / data.height();
        //    for(unsigned int w = 0; w < data.width(); w++) {
        //        data(h,w) *= 0.5f*(1.0f + cos(elevation));
        //    }
        //}
    }

    return Directivity::Create(data);
}

Directivity::Ptr Directivity::make_uniform(float amplitude, unsigned int oversampling)
{
    return Directivity::Create(Image<float>({oversampling, oversampling},
                                            HostVector<float>(1, 1.0f)));
}

Directivity::Ptr Directivity::disk_antenna(float diameter, float wavelength,
                                           const std::string& baffleMode,
                                           unsigned int oversampling)
{
    float period = 2.0f*wavelength / diameter;
    unsigned int N = std::max(64u, 8u*((unsigned int)(oversampling * M_PI / period)));

    HostImage data(N, N);
    for(unsigned int h = 0; h < data.height(); h++) {
        double elevation = M_PI * h / data.height();
        double cose2 = cos(elevation); cose2 *= cose2;
        double sine2 = sin(elevation); sine2 *= sine2;
        for(unsigned int w = 0; w < data.width(); w++) {
            double bearing = M_PI * w / data.width();
            double cosb2 = cos(bearing); cosb2 *= cosb2;
            double sinb2 = sin(bearing); sinb2 *= sinb2;

            double denom = (1 - sine2 * sinb2);
            double y2 = cose2 * sinb2 / denom;
            double z2 = sine2 * cosb2 / denom;
            double x = sqrt(1 - y2 - z2);
            if(abs(x) < 1.0e-2) {
                data(h,w) = 0.0f;
                continue;
            }
            if(elevation > 0.5*M_PI || bearing > 0.5f*M_PI)
                x = -x;

            double a = (M_PI*diameter / wavelength) * sin(atan2(sqrt(y2 + z2), x));
            if(abs(a) < 1.0e-3) {
                data(h,w) = 1.0f;
                continue;
            }
            data(h,w) = 2.0*j1(a) / a;
        }
    }

    if(baffleMode == "single-sided") {
        for(unsigned int h = 0; h < data.height(); h++) {
            for(unsigned int w = 0; w < data.width(); w++) {
                if(h > data.height() / 2 || w > data.width() / 2) {
                    data(h,w) = 0;
                }
            }
        }
    }
    else if(baffleMode == "cardioid") {
        throw std::runtime_error("baffleMode 'cardioid' unsupported");
        //for(unsigned int h = 0; h < data.height(); h++) {
        //    double elevation = M_PI * h / data.height();
        //    for(unsigned int w = 0; w < data.width(); w++) {
        //        data(h,w) *= 0.5f*(1.0f + cos(elevation));
        //    }
        //}
    }

    return Directivity::Create(data);
}

} //namespace simulation
} //namespace rtac
