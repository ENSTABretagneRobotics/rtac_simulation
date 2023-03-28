#include <rtac_simulation/SensorInstance1D.h>

#include <rtac_base/containers/Image.h>
#include <rtac_base/cuda/reductions.hcu>

namespace rtac { namespace simulation {

SensorInstance1D::SensorInstance1D(const SensorInfo::ConstPtr& info,
                                   const Pose& pose,
                                   float soundCelerity) :
    SensorInstance(info, pose, soundCelerity),
    info_(info)
{}

void SensorInstance1D::generate_psf_data()
{
    const auto& waveform = *waveform_;

    HostVector<Complex<float>> data(waveform.size());
    for(unsigned int i = 0; i < data.size(); i++) {
            data[i] = waveform[i];
    }

    waveformData_.set_image(data.size(), 1, (const float2*)data.data());
}

KernelView1D<Complex<float>> SensorInstance1D::kernel() const
{
    KernelView1D<Complex<float>> kernel;
    kernel.scaling_  = float2{1.0f / waveform_->duration()*soundCelerity_, 0.5f};
    kernel.function_ = waveformData_.texture();
    return kernel;
}

//template <typename Tin, unsigned int BlockSize = 512>
//__global__ void do_sparse_convolve_2d_f(ImageView<Complex<float>> out,
//                                        cuda::TextureVectorView<float> outBearings,
//                                        Linspace<float> outRanges,
//                                        const rtac::VectorView<const Tin>* bins,
//                                        KernelView1D<Complex<float>> kernel)
//{
//    // shared memory does not play well with templates
//    extern __shared__ __align__(sizeof(float)) unsigned char sharedMemory[];
//    float* sdata = reinterpret_cast<float*>(sharedMemory);
//
//    Complex<float> acc{0.0f,0.0f};
//    for(auto i = threadIdx.x; i < bins[blockIdx.y].size(); i += blockDim.x)
//    {
//        auto datum = bins[blockIdx.y][i];
//        // This multiplies the datum with the point spread function evaluated
//        // at the difference between the datum location and the output bin
//        // location.
//        acc += datum.value()*kernel(
//            datum.bearing() - outBearings[blockIdx.x],
//            datum.travel()  - outRanges[blockIdx.y]);
//    }
//    
//    //reduction of cuda::Complex must be performed independently on real and
//    //imag parts for now (issues with volatile conversion TODO:fix this)
//    sdata[threadIdx.x] = acc.real();
//    __syncthreads();
//    cuda::device::reduce_shared<float,BlockSize,rtac::cuda::Addition>(sdata, threadIdx.x);
//    if(threadIdx.x == 0)
//        acc.real(sdata[0]);
//
//    sdata[threadIdx.x] = acc.imag();
//    __syncthreads();
//    cuda::device::reduce_shared<float,BlockSize,rtac::cuda::Addition>(sdata, threadIdx.x);
//
//    if(threadIdx.x == 0) {
//        acc.imag(sdata[0]);
//        out(blockIdx.y,blockIdx.x) = acc;
//    }
//}

//void SensorInstance1D::do_reduce(Image<Complex<float>, cuda::DeviceVector>& out,
//    const cuda::DeviceVector<VectorView<const SimSample1D>>& bins) const
//{
//    out.resize({this->width(), this->height()});
//
//    static constexpr unsigned int BlockSize = 512;
//    uint3 grid{out.width(), out.height(), 1};
//
//    do_sparse_convolve_2d_f<<<grid, BlockSize, sizeof(float)*BlockSize>>>(
//        out.view(),
//        this->bearings_view(),
//        this->ranges(),
//        bins.data(),
//        this->kernel());
//
//    cudaDeviceSynchronize();
//    CUDA_CHECK_LAST();
//}


} //namespace simulation
} //namespace rtac
