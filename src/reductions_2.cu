#include <rtac_simulation/reductions_2.h>

#include <rtac_base/types/Complex.h>
#include <rtac_base/cuda/reductions.hcu>
#include <rtac_simulation/SensorModel.h>
#include <rtac_simulation/SensorInstance.h>


namespace rtac { namespace simulation {

template <typename D, typename Tin, typename KT, unsigned int BlockSize = 512>
__global__ void do_sparse_convolve_2d_f(D out,
                                        const rtac::VectorView<const Tin>* bins,
                                        KernelView2D<KT> kernel)
{
    // shared memory does not play well with templates
    using T = float;
    extern __shared__ __align__(sizeof(T)) unsigned char sharedMemory[];
    T* sdata = reinterpret_cast<T*>(sharedMemory);

    Complex<T> acc{0.0f,0.0f};
    for(auto i = threadIdx.x; i < bins[blockIdx.y].size(); i += blockDim.x)
    {
        auto datum = bins[blockIdx.y][i];
        // This multiplies the datum with the point spread function evaluated
        // at the difference between the datum location and the output bin
        // location.
        acc += datum.value()*kernel(
            datum.bearing() - out.width_dim() [blockIdx.x],
            datum.travel()  - out.height_dim()[blockIdx.y]);
    }
    
    //reduction of cuda::Complex must be performed independently on real and
    //imag parts for now (issues with volatile conversion TODO:fix this)
    sdata[threadIdx.x] = acc.real();
    __syncthreads();
    cuda::device::reduce_shared<T,BlockSize,rtac::cuda::Addition>(sdata, threadIdx.x);
    if(threadIdx.x == 0)
        acc.real(sdata[0]);

    sdata[threadIdx.x] = acc.imag();
    __syncthreads();
    cuda::device::reduce_shared<T,BlockSize,rtac::cuda::Addition>(sdata, threadIdx.x);

    if(threadIdx.x == 0) {
        acc.imag(sdata[0]);
        out(blockIdx.y,blockIdx.x) = acc;
    }
}

void sparse_convolve_2d(SensorModel2D<Complex<float>, float>& out,
                        const cuda::DeviceVector<VectorView<const SimSample2D>>& bins)
{
    if(out.height() != bins.size()) {
        throw std::runtime_error("Inconsistent sizes");
    }

    static constexpr unsigned int BlockSize = 512;
    uint3 grid{out.width(), out.height(), 1};
    do_sparse_convolve_2d_f<<<grid, BlockSize, sizeof(float)*BlockSize>>>(
        out.data().view(), bins.data(), out.point_spread_function().view());
    cudaDeviceSynchronize();
    CUDA_CHECK_LAST();
}

void sparse_convolve_2d(SensorModel2D_2<Complex<float>>& out,
                        const cuda::DeviceVector<VectorView<const SimSample2D>>& bins)
{
    if(out.height() != bins.size()) {
        throw std::runtime_error("Inconsistent sizes");
    }
    static constexpr unsigned int BlockSize = 512;
    uint3 grid{out.width(), out.height(), 1};

    if(out.point_spread_function()->is_complex()) {
        do_sparse_convolve_2d_f<<<grid, BlockSize, sizeof(float)*BlockSize>>>(
            out.data().view(), bins.data(),
            out.point_spread_function()->complex_cast()->kernel());
    }
    else {
        do_sparse_convolve_2d_f<<<grid, BlockSize, sizeof(float)*BlockSize>>>(
            out.data().view(), bins.data(),
            out.point_spread_function()->real_cast()->kernel());
    }

    cudaDeviceSynchronize();
    CUDA_CHECK_LAST();
}

template <typename Tin, typename KT, unsigned int BlockSize = 512>
__global__ void do_sparse_convolve_2d_f(ImageView<Complex<float>> out,
                                        cuda::TextureVectorView<float> outBearings,
                                        Linspace<float> outRanges,
                                        const rtac::VectorView<const Tin>* bins,
                                        KernelView2D<KT> kernel)
{
    // shared memory does not play well with templates
    using T = float;
    extern __shared__ __align__(sizeof(T)) unsigned char sharedMemory[];
    T* sdata = reinterpret_cast<T*>(sharedMemory);

    Complex<T> acc{0.0f,0.0f};
    for(auto i = threadIdx.x; i < bins[blockIdx.y].size(); i += blockDim.x)
    {
        auto datum = bins[blockIdx.y][i];
        // This multiplies the datum with the point spread function evaluated
        // at the difference between the datum location and the output bin
        // location.
        acc += datum.value()*kernel(
            datum.bearing() - outBearings[blockIdx.x],
            datum.travel()  - outRanges[blockIdx.y]);
    }
    
    //reduction of cuda::Complex must be performed independently on real and
    //imag parts for now (issues with volatile conversion TODO:fix this)
    sdata[threadIdx.x] = acc.real();
    __syncthreads();
    cuda::device::reduce_shared<T,BlockSize,rtac::cuda::Addition>(sdata, threadIdx.x);
    if(threadIdx.x == 0)
        acc.real(sdata[0]);

    sdata[threadIdx.x] = acc.imag();
    __syncthreads();
    cuda::device::reduce_shared<T,BlockSize,rtac::cuda::Addition>(sdata, threadIdx.x);

    if(threadIdx.x == 0) {
        acc.imag(sdata[0]);
        out(blockIdx.y,blockIdx.x) = acc;
    }
}

void sparse_convolve_2d(SensorModel2D_Base& out,
                        const cuda::DeviceVector<VectorView<const SimSample2D>>& bins)
{
    if(out.height() != bins.size()) {
        throw std::runtime_error("Inconsistent sizes");
    }
    static constexpr unsigned int BlockSize = 512;
    uint3 grid{out.width(), out.height(), 1};

    auto& outTmp = dynamic_cast<SensorModel2D_Complex&>(out);

    //if(out.point_spread_function()->is_complex()) {
    //    do_sparse_convolve_2d_f<<<grid, BlockSize, sizeof(float)*BlockSize>>>(
    //        outTmp.data_view(), 
    //        out.bearings_view(),
    //        out.ranges(),
    //        bins.data(),
    //        outTmp.point_spread_function()->complex_cast()->kernel());
    //}
    //else {
    //    do_sparse_convolve_2d_f<<<grid, BlockSize, sizeof(float)*BlockSize>>>(
    //        outTmp.data_view(), 
    //        out.bearings_view(),
    //        out.ranges(),
    //        bins.data(),
    //        outTmp.point_spread_function()->real_cast()->kernel());
    //}
    do_sparse_convolve_2d_f<<<grid, BlockSize, sizeof(float)*BlockSize>>>(
        outTmp.data_view(), 
        out.bearings_view(),
        out.ranges(),
        bins.data(),
        outTmp.point_spread_function()->kernel());

    cudaDeviceSynchronize();
    CUDA_CHECK_LAST();
}

void sparse_convolve_2d(Image<Complex<float>, cuda::DeviceVector>& out,
                        const SensorInstance& sensor,
                        const cuda::DeviceVector<VectorView<const SimSample2D>>& bins)
{
    if(sensor.height() != bins.size()) {
        throw std::runtime_error("Inconsistent sizes");
    }
    out.resize({sensor.width(), sensor.height()});

    static constexpr unsigned int BlockSize = 512;
    uint3 grid{out.width(), out.height(), 1};

    do_sparse_convolve_2d_f<<<grid, BlockSize, sizeof(float)*BlockSize>>>(
        out.view(),
        sensor.bearings_view(),
        sensor.ranges(),
        bins.data(),
        sensor.kernel());

    cudaDeviceSynchronize();
    CUDA_CHECK_LAST();
}


} //namespace simulation
} //namespace rtac
