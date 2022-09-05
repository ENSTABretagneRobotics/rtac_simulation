#ifndef _DEF_RTAC_SIMULATION_FUNCTION_2D_H_
#define _DEF_RTAC_SIMULATION_FUNCTION_2D_H_

#include <vector>

#include <rtac_base/cuda_defines.h>
#include <rtac_base/types/Image.h>
#include <rtac_base/cuda/Texture2D.h>
#include <rtac_base/cuda/DeviceVector.h>

namespace rtac { namespace simulation {

template <typename T>
struct Function2DView
{
    cudaTextureObject_t handle_;
    float2 xScaling_;
    float2 yScaling_;

    #ifdef RTAC_CUDACC
    __device__ T operator()(float x, float y) const {
        return tex2D<T>(handle_,
                        xScaling_.x*x + xScaling_.y,
                        yScaling_.x*y + yScaling_.y);
    }
    #endif
};

/**
 * This represent a continuous 2D function defined on 2D rectangle. It allows
 * for fast function evaluation on the gpu.
 *
 * The function is encoded into a texture on the GPU. This implies that the
 * function poutput type T should be either float, float2 or float4.
 */
template <typename T>
class Function2D
{
    public:

    using Bounds      = rtac::types::Interval<float>;
    using DeviceImage = rtac::types::Image<T,rtac::cuda::DeviceVector>;
    using HostImage   = rtac::types::Image<T,std::vector>;

    protected:

    rtac::cuda::Texture2D<T> data_;
    Bounds xBounds_;
    Bounds yBounds_;

    public:
    
    Function2D(const Bounds& xBounds, const Bounds& yBounds,
               const DeviceImage& data);
    Function2D(const Bounds& xBounds, const Bounds& yBounds,
               const HostImage& data);

    Bounds x_bounds() const { return xBounds_; }
    Bounds y_bounds() const { return yBounds_; }

    float width()  const { return xBounds_.length(); }
    float height() const { return yBounds_.length(); }

    const cuda::Texture2D<T>& data() const { return data_; }
    cuda::Texture2D<T>&       data()       { return data_; }

    void set_bounds(Bounds xBounds, Bounds yBounds);
    void set_bounds(float xmin, float xmax, float ymin, float ymax);
    void set_bounds(float width, float height);

    Function2DView<T> view() const;
};

template <typename T>
Function2D<T>::Function2D(const Bounds& xBounds, const Bounds& yBounds,
                          const DeviceImage& data) :
    xBounds_(xBounds), yBounds_(yBounds)
{
    data_.set_image(data.width(), data.height(), data.container());
    data_.set_filter_mode(cuda::Texture2D<T>::FilterLinear, false);
    data_.set_wrap_mode(cuda::Texture2D<T>::WrapBorder, true);
}

template <typename T>
Function2D<T>::Function2D(const Bounds& xBounds, const Bounds& yBounds,
                          const HostImage& data) :
    xBounds_(xBounds), yBounds_(yBounds)
{
    data_.set_image(data.width(), data.height(), data.data());
    data_.set_filter_mode(cuda::Texture2D<T>::FilterLinear, false);
    data_.set_wrap_mode(cuda::Texture2D<T>::WrapBorder, true);
}

template <typename T>
void Function2D<T>::set_bounds(Bounds xBounds, Bounds yBounds)
{
    xBounds_ = xBounds;
    yBounds_ = yBounds;
}

template <typename T>
void Function2D<T>::set_bounds(float xmin, float xmax, float ymin, float ymax)
{
    xBounds_ = Bounds{xmin,xmax};
    yBounds_ = Bounds{ymin,ymax};
}

template <typename T>
void Function2D<T>::set_bounds(float width, float height)
{
    xBounds_ = Bounds{-0.5f*width,  0.5f*width};
    yBounds_ = Bounds{-0.5f*height, 0.5f*height};
}

template <typename T>
Function2DView<T> Function2D<T>::view() const
{
    Function2DView<T> view;
    if(data_.width() == 1) {
        // should not happen
        view.xScaling_.x = 0.0f;
        view.xScaling_.y = 0.0f;
    }
    else {
        view.xScaling_.x =          1.0f / xBounds_.length();
        view.xScaling_.y = -xBounds_.min / xBounds_.length();
    }
    if(data_.height() == 1) {
        // should not happen
        view.yScaling_.x = 0.0f;
        view.yScaling_.y = 0.0f;
    }
    else {
        view.yScaling_.x =          1.0f / yBounds_.length();
        view.yScaling_.y = -yBounds_.min / yBounds_.length();
    }
    view.handle_ = data_.texture();

    return view;
}

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_FUNCTION_2D_H_
