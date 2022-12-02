#ifndef _DEF_RTAC_SIMULATION_POLAR_KERNEL_2D_H_
#define _DEF_RTAC_SIMULATION_POLAR_KERNEL_2D_H_

#include <memory>

#include <rtac_base/types/Image.h>
#include <rtac_base/cuda/DeviceVector.h>
#include <rtac_base/cuda/Texture2D.h>

namespace rtac { namespace simulation {

template <typename T>
struct PolarKernelView2D
{
    float2 bearingScaling_;
    float2 rangeScaling_;
    cudaTextureObject_t function_;

    #ifdef RTAC_CUDACC
    __device__ T operator()(float range, float bearing) const {
        return tex2D<T>(function_,
                        fmaf(bearingScaling_.x, bearing, bearingScaling_.y),
                        fmaf(rangeScaling_.x,   range,   rangeScaling_.y));
    }
    #endif
};

template <typename T>
class PolarKernel2D
{
    public:

    using Ptr         = std::shared_ptr<PolarKernel2D<T>>;
    using ConstPtr    = std::shared_ptr<const PolarKernel2D<T>>;

    using DeviceImage = rtac::Image<T, cuda::DeviceVector>;

    protected:

    float bearingSpan_;
    float rangeSpan_;
    cuda::Texture2D<T> function_;

    PolarKernel2D(float bearingSpan, 
                  float rangeSpan,
                  const DeviceImage& data) :
        bearingSpan_(bearingSpan), rangeSpan_(rangeSpan)
    {
        function_.set_filter_mode(cuda::Texture2D<T>::FilterLinear, false);
        function_.set_wrap_mode(cuda::Texture2D<T>::WrapBorder, true);
        function_.set_image(data.width(), data.height(), data.container());
    }

    public:

    static Ptr Create(float bearingSpan,
                      float rangeSpan,
                      const DeviceImage& data)
    {
        return Ptr(new PolarKernel2D<T>(bearingSpan, rangeSpan, data));
    }

    const cuda::Texture2D<T>& texture() const { return function_; }
    float bearing_span() const { return bearingSpan_; }
    float range_span()   const { return rangeSpan_; }

    PolarKernelView2D<T> view() const  {
        return PolarKernelView2D<T>{
            float2{1.0f / bearingSpan_, 0.5f},
            float2{1.0f / rangeSpan_,   0.5f},
            function_.texture()};
    }
};

} //namespace simulation
} //namespace rtac



#endif //_DEF_RTAC_SIMULATION_POLAR_KERNEL_2D_H_
