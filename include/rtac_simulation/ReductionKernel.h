#ifndef _DEF_RTAC_SIMULATION_REDUCTION_KERNEL_H_
#define _DEF_RTAC_SIMULATION_REDUCTION_KERNEL_H_

#include <memory>

#include <rtac_base/containers/Image.h>
#include <rtac_base/cuda/DeviceVector.h>
#include <rtac_base/cuda/Texture2D.h>

namespace rtac { namespace simulation {

template <typename T>
struct KernelView1D
{
    float2              scaling_;
    cudaTextureObject_t function_;

    #ifdef RTAC_CUDACC
    __device__ T operator()(float x) const {
        return tex1D<T>(function_, fmaf(scaling_.x, x, scaling_.y));
    }
    #endif //RTAC_CUDACC
};

template <typename T>
struct KernelView2D
{
    float2 xScaling_;
    float2 yScaling_;
    cudaTextureObject_t function_;

    #ifdef RTAC_CUDACC
    __device__ T operator()(float x, float y) const {
        return tex2D<T>(function_,
                        fmaf(xScaling_.x, x, xScaling_.y),
                        fmaf(yScaling_.x, y, yScaling_.y));

    }
    #endif //RTAC_CUDACC
};

template <typename T>
class Kernel2D
{
    public:

    using Ptr      = std::shared_ptr<Kernel2D<T>>;
    using ConstPtr = std::shared_ptr<const Kernel2D<T>>;

    using DeviceImage = rtac::Image<T, cuda::DeviceVector>;

    protected:

    float xSpan_;
    float ySpan_;
    cuda::Texture2D<T> function_;

    public:

    template <template<typename>class VectorT>
    Kernel2D(float xSpan, float ySpan,
             const rtac::Image<T,VectorT>& data) :
        xSpan_(xSpan), ySpan_(ySpan)
    {
        function_.set_filter_mode(cuda::Texture2D<T>::FilterLinear, false);
        function_.set_wrap_mode(cuda::Texture2D<T>::WrapBorder, true);
        function_.set_image(data.width(), data.height(), data.container());
    }

    const cuda::Texture2D<T>& texture() const { return function_; }
    float x_span() const { return xSpan_; }
    float y_span() const { return ySpan_; }

    unsigned int width()  const { return function_.width();  }
    unsigned int height() const { return function_.height(); }
    unsigned int size()   const { return function_.size();   }

    KernelView2D<T> view() const  {
        KernelView2D<T> res;
        res.xScaling_ = float2{1.0f / xSpan_, 0.5f};
        res.yScaling_ = float2{1.0f / ySpan_, 0.5f};
        res.function_ = function_.texture();
        return res;
    }
};

Image<float,cuda::DeviceVector> render_kernel(const Kernel2D<float>& kernel);

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_REDUCTION_KERNEL_H_
