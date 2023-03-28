#ifndef _DEF_RTAC_SIMULATION_REDUCTION_KERNEL_H_
#define _DEF_RTAC_SIMULATION_REDUCTION_KERNEL_H_

#include <memory>

#include <rtac_base/containers/Image.h>
#include <rtac_base/cuda/DeviceVector.h>
#include <rtac_base/cuda/Texture2D.h>
#include <rtac_base/cuda/vec_math.h>

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


template <>
struct KernelView1D<Complex<float>>
{
    float2              scaling_;
    cudaTextureObject_t function_;

    #ifdef RTAC_CUDACC
    __device__ Complex<float> operator()(float x) const {
        return make_complex(tex1D<float2>(function_, fmaf(scaling_.x, x, scaling_.y)));
    }
    #endif //RTAC_CUDACC
};


template <>
struct KernelView2D<Complex<float>>
{
    float2 xScaling_;
    float2 yScaling_;
    cudaTextureObject_t function_;

    #ifdef RTAC_CUDACC
    __device__ Complex<float> operator()(float x, float y) const {
        return make_complex(tex2D<float2>(function_,
                                          fmaf(xScaling_.x, x, xScaling_.y),
                                          fmaf(yScaling_.x, y, yScaling_.y)));

    }
    #endif //RTAC_CUDACC
};


} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_REDUCTION_KERNEL_H_
