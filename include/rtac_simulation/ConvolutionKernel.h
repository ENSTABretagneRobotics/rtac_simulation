#ifndef _DEF_RTAC_SIMULATION_CONVOLUTION_KERNEL_H_
#define _DEF_RTAC_SIMULATION_CONVOLUTION_KERNEL_H_

#include <memory>

#include <rtac_base/types/Bounds.h>

#include <rtac_base/cuda/Texture2D.h>
#include <rtac_base/cuda/TextureView2D.h>

namespace rtac { namespace simulation {

/**
 * This type allows for fast memory access to the convolution kernel during a
 * sparse convolution operation.
 *
 * On the GPU, texture fetch are (nearly) always done using coordinates between
 * 0 and 1. This type perform a rescaling on the input coordinates before
 * texture fetch so the user can use coordinates expressed in their own system.
 */
template <typename T>
struct KernelView2D
{
    float2              xScaling_;
    float2              yScaling_;
    cudaTextureObject_t handle_;

    #ifdef RTAC_CUDACC
    __device__ T operator()(float x, float y) const {
        return tex2D<T>(handle_,
                        xScaling_.x*x + xScaling_.y,
                        yScaling_.x*y + yScaling_.y);
    }
    #endif //RTAC_CUDACC
};

/**
 * The ConvolutionKernel2D represent a 2D function to be used as a convolution
 * kernel in the function sparse convolve 2D.
 *
 * For now kernel is assumed centered so kernel function support is
 * parameterized with a width and height attribute.
 */
template <typename T>
class ConvolutionKernel2D
{
    public:

    using Ptr      = std::shared_ptr<ConvolutionKernel2D<T>>;
    using ConstPtr = std::shared_ptr<const ConvolutionKernel2D<T>>;

    using Bounds = rtac::Bounds<float>;

    protected:
    
    Bounds xBounds_;
    Bounds yBounds_;
    cuda::Texture2D<T> function_;

    ConvolutionKernel2D();

    public:
    
    static Ptr Create() { return Ptr(new ConvolutionKernel2D<T>()); }

    Bounds x_bounds() const { return xBounds_; }
    Bounds y_bounds() const { return yBounds_; }

    float width()  const { return xBounds_.length(); }
    float height() const { return yBounds_.length(); }

    const cuda::Texture2D<T>& function() const { return function_; }
    cuda::Texture2D<T>&       function()       { return function_; }

    void set_bounds(Bounds xBounds, Bounds yBounds);
    void set_bounds(float xmin, float xmax, float ymin, float ymax);
    void set_bounds(float width, float height);

    KernelView2D<T> view() const;
};

template <typename T>
ConvolutionKernel2D<T>::ConvolutionKernel2D()
{
    function_.set_filter_mode(cuda::Texture2D<T>::FilterLinear, false);
    function_.set_wrap_mode(cuda::Texture2D<T>::WrapBorder, true);
}

template <typename T>
void ConvolutionKernel2D<T>::set_bounds(Bounds xBounds, Bounds yBounds)
{
    xBounds_ = xBounds;
    yBounds_ = yBounds;
}

template <typename T>
void ConvolutionKernel2D<T>::set_bounds(float xmin, float xmax, float ymin, float ymax)
{
    xBounds_ = Bounds{xmin,xmax};
    yBounds_ = Bounds{ymin,ymax};
}

template <typename T>
void ConvolutionKernel2D<T>::set_bounds(float width, float height)
{
    xBounds_ = Bounds{-0.5f*width,  0.5f*width};
    yBounds_ = Bounds{-0.5f*height, 0.5f*height};
}

template <typename T>
KernelView2D<T> ConvolutionKernel2D<T>::view() const
{
    KernelView2D<T> view;
    if(function_.width() == 1) {
        // should not happen
        view.xScaling_.x = 0.0f;
        view.xScaling_.y = 0.0f;
    }
    else {
        view.xScaling_.x =          1.0f / xBounds_.length();
        view.xScaling_.y = -xBounds_.min / xBounds_.length();
    }
    if(function_.height() == 1) {
        // should not happen
        view.yScaling_.x = 0.0f;
        view.yScaling_.y = 0.0f;
    }
    else {
        view.yScaling_.x =          1.0f / yBounds_.length();
        view.yScaling_.y = -yBounds_.min / yBounds_.length();
    }
    view.handle_ = function_.texture();

    return view;
}

}; //namespace simulation
}; //namespace rtac

#endif //_DEF_RTAC_SIMULATION_CONVOLUTION_KERNEL_H_


