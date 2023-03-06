#ifndef _DEF_RTAC_SIMULATION_DIRECTIVITY_FUNCTION_H_
#define _DEF_RTAC_SIMULATION_DIRECTIVITY_FUNCTION_H_

#include <iostream>
#include <memory>
#include <cmath>

#include <rtac_base/containers/Image.h>

#include <rtac_base/cuda_defines.h>
#include <rtac_base/cuda/Texture2D.h>
#include <rtac_base/cuda/vec_math.h>

namespace rtac { namespace simulation {

/**
 * The directivity is considered symetric both in bearing and elevation.
 */
struct DirectivityView
{
    cudaTextureObject_t handle_;
    float xScaling_; // most of the time equal to 1.0 / M_PI
    float yScaling_; // most of the time equal to 1.0 / M_PI

    #ifdef RTAC_CUDACC
    /**
     * Reads a value from the directivity function. The direction is given by a
     * 3D unit vector, {1,0,0} is the origin.
     */
    __device__ float operator()(const float3& dir) const {
        return tex2D<float>(handle_,
                        xScaling_*atan2(dir.y, dir.x),
                        yScaling_*atan2(dir.z, dir.x));
    }
    #endif //RTAC_CUDACC
};

/**
 * This represent an antenna directivity function. The directivity is
 * considered symetric both in bearing (around the .z axis) and elevation
 * (around the .y axis).
 */
class Directivity
{
    public:

    using Ptr      = std::shared_ptr<Directivity>;
    using ConstPtr = std::shared_ptr<const Directivity>;

    using DeviceImage = rtac::Image<float, rtac::cuda::DeviceVector>;
    using HostImage   = rtac::Image<float, rtac::HostVector>;
    using DataShape   = typename HostImage::Shape;

    protected:

    cuda::Texture2D<float> texture_;

    Directivity(const HostImage& data);
    Directivity(const DeviceImage& data);

    public:

    static Ptr Create(const HostImage& data)   { return Ptr(new Directivity(data)); }
    static Ptr Create(const DeviceImage& data) { return Ptr(new Directivity(data)); }

    void load_default_configuration();

    cuda::Texture2D<float>&       texture()       { return texture_; }
    const cuda::Texture2D<float>& texture() const { return texture_; }
    
    DirectivityView view() const;

    static Ptr from_sinc_parameters(float bearingAperture,
                                    float elevationAperture,
                                    unsigned int oversampling = 8);
    static Ptr rectangle_antenna(float width, float height, float wavelength,
                                 const std::string& baffleMode = "single-sided",
                                 unsigned int oversampling = 8);
};

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_DIRECTIVITY_FUNCTION_H_
