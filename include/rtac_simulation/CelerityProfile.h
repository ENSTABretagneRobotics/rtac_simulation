#ifndef _DEF_RTAC_SIMULATION_CELERITY_PROFILE_H_
#define _DEF_RTAC_SIMULATION_CELERITY_PROFILE_H_

#include <memory>

#include <rtac_base/cuda_defines.h>
#include <rtac_base/types/Bounds.h>
#include <rtac_base/cuda/Texture2D.h>

namespace rtac { namespace simulation {

/**
 * CelerityProfile to be used on cuda functions.
 */
struct CelerityProfileView
{
    float a_, b_;
    cudaTextureObject_t celerity_;
    cudaTextureObject_t gradient_;

    #ifdef RTAC_CUDACC
    __device__ float get_celerity(float z) const {
        return tex2D<float>(celerity_, fmaf(a_, z, b_), 0.0f);
    }
    __device__ float get_gradient(float z) const {
        return tex2D<float>(gradient_, fmaf(a_, z, b_), 0.0f);
    }
    __device__ float3 get_direction_delta(const float3& position,
                                          const float3& direction) const
    {
        float x = fmaf(a_, position.z, b_);
        float c = tex2D<float>(celerity_, x, 0.0f);
        float g = tex2D<float>(gradient_, x, 0.0f);

        float3 delta = -c * g*direction.z * direction;
        delta.z += g*c;

        return delta;
        
        // above equivalent to the following.
        //float  x = fmaf(a_, position.z, b_);
        //float  c = tex2D<float>(celerity_, x, 0.0f);
        //float3 g = float3({0.0f,0.0f,tex2D<float>(gradient_, x, 0.0f)});
        //return c*(g - dot(g, direction)*direction);
    }
    #endif //RTAC_CUDACC
};

/**
 * This class implements a 1D function representing the celerity profile in the
 * water column.
 *
 * The Celerity profile and its gradient are stored in a Texture on the GPU for
 * fast access.
 */
class CelerityProfile
{
    public:

    using Ptr      = std::shared_ptr<CelerityProfile>;
    using ConstPtr = std::shared_ptr<const CelerityProfile>;

    protected:

    Bounds<float> zBounds_;
    rtac::cuda::Texture2D<float> celerity_;
    rtac::cuda::Texture2D<float> gradient_;

    CelerityProfile(unsigned int inputSize,
                    const float* heightData,
                    const float* celerityData,
                    unsigned int profileSize);

    public:

    static Ptr Create(unsigned int inputSize,
                      const float* heightData,
                      const float* celerityData,
                      unsigned int profileSize);
    
    CelerityProfileView view() const;

    Bounds<float> z_bounds() const { return zBounds_; }
    const rtac::cuda::Texture2D<float>& celerity() const { return celerity_; }
    const rtac::cuda::Texture2D<float>& gradient() const { return gradient_; }
};

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_CELERITY_PROFILE_H_
