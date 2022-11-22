#include <rtac_simulation/helpers/PolarTargetRenderer.h>

#include <rtac_base/cuda_defines.h>

namespace rtac { namespace display {

PolarTargetRenderer::PolarTargetRenderer(const GLContext::Ptr& context) :
    FanRenderer(context)
{
    this->set_direction(FanRenderer::Direction::Down);
}

void PolarTargetRenderer::set_data(rtac::simulation::PolarTarget2D<float>::ConstPtr data)
{
    this->set_geometry<float>(data);

    tmpData_ = data->data().container();
    this->FanRenderer::set_data(
        Shape{data->bearing_count(), data->range_count()}, tmpData_);
}

__global__ void render_complex_data(
    rtac::VectorView<const cuda::Complex<float>> in, float* out)
{
    auto idx = blockDim.x*blockIdx.x + threadIdx.x;
    for(; idx < in.size(); idx += gridDim.x*blockDim.x) {
        out[idx] = log(1.0f + 100.0f*abs(in[idx]));
    }
}

void PolarTargetRenderer::set_data(
    rtac::simulation::PolarTarget2D<cuda::Complex<float>>::ConstPtr data)
{
    this->set_geometry<cuda::Complex<float>>(data);

    tmpData_.resize(data->data().container().size());
    {
        rtac::VectorView<const cuda::Complex<float>> dataView(tmpData_.size(),
            data->data().container().data());
        auto mappedPtr = tmpData_.map_cuda();

        render_complex_data<<<tmpData_.size() / RTAC_BLOCKSIZE, RTAC_BLOCKSIZE>>>(
            dataView, mappedPtr);
        cudaDeviceSynchronize();
        CUDA_CHECK_LAST();
    }
    this->FanRenderer::set_data(
        Shape{data->bearing_count(), data->range_count()}, tmpData_);
}

} //namespace display
} //namespace rtac
