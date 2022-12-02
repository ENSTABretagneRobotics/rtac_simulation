#ifndef _DEF_RTAC_SIMULATION_POLAR_RECEIVER_2D_H_
#define _DEF_RTAC_SIMULATION_POLAR_RECEIVER_2D_H_

#include <iostream>
#include <memory>

#include <rtac_base/types/Complex.h>

#include <rtac_base/cuda/DeviceVector.h>
#include <rtac_base/cuda/HostVector.h>

#include <rtac_simulation/Sample.h>
#include <rtac_simulation/Receiver.h>
#include <rtac_simulation/RangeBinner.h>
#include <rtac_simulation/PolarKernel2D.h>
#include <rtac_simulation/PolarTarget2D.h>

#ifdef RTAC_CUDACC

#include <rtac_base/cuda/reductions.hcu>

#endif //RTAC_CUDACC

namespace rtac { namespace simulation {

template <typename T>
class PolarReceiver2D : public Receiver<PolarSample2D<T>>
{
    public:

    using Ptr      = std::shared_ptr<PolarReceiver2D<T>>;
    using ConstPtr = std::shared_ptr<const PolarReceiver2D<T>>;

    using Kernel = PolarKernel2D<T>;
    using Target = PolarTarget2D<Complex<T>>;
    using Bin    = VectorView<PolarSample2D<T>>;

    protected:

    typename Kernel::ConstPtr psf_;
    typename Target::Ptr      target_;

    RangeBinner           binner_;
    cuda::HostVector<Bin> bins_;

    PolarReceiver2D(Directivity::ConstPtr directivity,
                    typename Kernel::ConstPtr psf,
                    typename Target::Ptr target) :
        Receiver<PolarSample2D<T>>(directivity),
        psf_(psf),
        target_(target)
    {}

    void do_reduce();

    public:

    static Ptr Create(Directivity::ConstPtr directivity,
                      typename Kernel::ConstPtr kernel,
                      typename Target::Ptr target)
    {
        return Ptr(new PolarReceiver2D<T>(directivity, kernel, target));
    }

    typename Target::Ptr      target()       { return target_; }
    typename Target::ConstPtr target() const { return target_; }

    typename Kernel::ConstPtr psf() const { return psf_; }

    void reduce_samples();
};

/**
 * This method updates the target image from the samples filled-in by the
 * insonification step.
 *
 * This is a costly operation akind to a convolution but computed without a
 * grid. It can be viewed as a convolution with a regular kernel but input data
 * is sparse and output data is a non-uniformly sampled grid.
 */
template <typename T>
void PolarReceiver2D<T>::reduce_samples()
{
    int binOverlap = psf_->range_span() / target_->range_resolution();

    binner_.update_ranges(*target_);
    binner_.compute_bins(this->samples(), bins_, binOverlap);
    this->do_reduce();
}

#ifdef RTAC_CUDACC

template <typename T, unsigned int BlockSize = RTAC_BLOCKSIZE>
__global__ void do_reduce_polar_2d(const VectorView<PolarSample2D<T>>* bins,
                                   PolarKernelView2D<T> kernel,
                                   PolarTargetView2D<Complex<T>> target)
{
    // shared memory does not play well with templates
    extern __shared__ __align__(sizeof(T)) unsigned char sharedMemory[];
    T* sdata = reinterpret_cast<T*>(sharedMemory);

    Complex<T> res{0.0f,0.0f};
    for(auto i = threadIdx.x; i < bins[blockIdx.x].size(); i += blockDim.x)
    {
        auto sample = bins[blockIdx.x][i];
        // This multiplies the datum with the point spread function evaluated
        // at the difference between the datum location and the output bin
        // location.
        res += sample.datum*kernel(sample.range() - target.range(blockIdx.x),
                                   sample.theta() - target.bearing(blockIdx.y));
    }
    
    //reduction of cuda::Complex must be performed independently on real and
    //imag parts for now (issues with volatile conversion TODO:fix this)
    sdata[threadIdx.x] = res.real();
    __syncthreads();
    cuda::device::reduce_shared<T,BlockSize,rtac::cuda::Addition>(sdata, threadIdx.x);
    if(threadIdx.x == 0)
        res.real(sdata[0]);

    sdata[threadIdx.x] = res.imag();
    __syncthreads();
    cuda::device::reduce_shared<T,BlockSize,rtac::cuda::Addition>(sdata, threadIdx.x);

    if(threadIdx.x == 0) {
        res.imag(sdata[0]);
        target(blockIdx.x, blockIdx.y) = res;
    }
}

template <typename T>
void PolarReceiver2D<T>::do_reduce()
{
    cuda::DeviceVector<Bin> bins(bins_);
    dim3 gridDim((uint32_t)target_->range_count(), 
                 (uint32_t)target_->bearing_count(), 1);
    do_reduce_polar_2d<<<gridDim, RTAC_BLOCKSIZE,
                         sizeof(T)*RTAC_BLOCKSIZE>>>(
        bins.data(), psf_->view(), target_->view());
    cudaDeviceSynchronize();
    CUDA_CHECK_LAST();
}

#endif

} //namespace simulation
} //namespace simulation

#endif //_DEF_RTAC_SIMULATION_POLAR_RECEIVER_2D_H_
