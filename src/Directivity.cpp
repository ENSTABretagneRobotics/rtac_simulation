#include <rtac_simulation/Directivity.h>

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
                                                   DataShape shape)
{
    // sin(SincHalf) / SincHalf = 0.5 (-3dB on directivity)
    //constexpr float SincHalf = 1.89549;
    constexpr float SincHalf = 1.39156f;

    auto optimal_size = [](float aperture, int oversampling) {
        float shannonPeriod = 0.25f*M_PIf*aperture / SincHalf;
        unsigned int N = 2;
        for(; M_PIf*oversampling  > shannonPeriod*N; N *= 2);
        return N;
    };

    auto sinc_sampling = [](unsigned int N, float aperture, int n) {
        if(n == 0) return 1.0f;
        float x = 2.0f*SincHalf*(M_PIf*n) / (N*aperture);
        return std::sin(x) / x;
    };

    int oversampling = 8;
    if(shape.area() == 0) {
        // No size parameter given by user. Finding one.
        shape.width  = optimal_size(bearingAperture,   oversampling);
        shape.height = optimal_size(elevationAperture, oversampling);
        std::cout << "Optimal shape : " << shape << std::endl;
    }

    HostImage data(shape);
    for(int h = 0; h < shape.height; h++) {
        float tmp = sinc_sampling(shape.height, elevationAperture, h);
        for(int w = 0; w < shape.width; w++) {
            data(h,w) = tmp*sinc_sampling(shape.width, bearingAperture, w);
        }
    }

    return Directivity::Create(data);
}

} //namespace simulation
} //namespace rtac
