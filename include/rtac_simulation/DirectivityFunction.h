#ifndef _DEF_RTAC_SIMULATION_DIRECTIVITY_FUNCTION_H_
#define _DEF_RTAC_SIMULATION_DIRECTIVITY_FUNCTION_H_

#include <memory>

#include <rtac_base/cuda_defines.h>
#include <rtac_base/cuda/Texture2D.h>

namespace rtac { namespace simulation {

/**
 * The directivity is considered symetric both in bearing and elevation.
 */
template <typename T>
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
    __device__ T operator()(const float3& dir) const {
        return tex2D<T>(handle_,
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
template <typename T>
class Directivity
{
    public:

    using Ptr      = std::shared_ptr<Directivity<T>>;
    using ConstPtr = std::shared_ptr<const Directivity<T>>;

    using DeviceImage = rtac::types::Image<T,rtac::cuda::DeviceVector>;
    using HostImage   = rtac::types::Image<T,std::vector>;

    protected:

    cuda::Texture2D<T> texture_;

    Directivity(const HostImage& data);
    Directivity(const DeviceImage& data);

    public:

    static Ptr Create(const HostImage& data)   { return Ptr(new Directivity<T>(data)); }
    static Ptr Create(const DeviceImage& data) { return Ptr(new Directivity<T>(data)); }

    void load_default_configuration();

    cudaTexture2D<T>&       texture()       { return texture_; }
    const cudaTexture2D<T>& texture() const { return texture_; }
    
    DirectivityView<T> view() const;
};

template <typename T>
Directivity<T>::Directivity(const HostImage& data)
{
    this->load_default_configuration();
    texture_->set_image(data.width(), data.height(), data.data());
}

template <typename T>
Directivity<T>::Directivity(const DeviceImage& data)
{
    this->load_default_configuration();
    texture_->set_image(data.width(), data.height(), data.container());
}

template <typename T>
void Directivity<T>::load_default_configuration()
{
    texture_.set_filter_mode(cuda::Texture2D<T>::FilterLinear, false);
    texture_.set_wrap_mode(cuda::Texture2D<T>::WrapMirror, true);
}

template <typename T>
DirectivityView<T> Directivity<T>::view() const
{
    return DirectivityView<T>({texture_.texture(), 0.5f / M_PI, 0.5f / M_PI});
}

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_DIRECTIVITY_FUNCTION_H_
