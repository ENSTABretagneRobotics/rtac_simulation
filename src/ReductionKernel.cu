#include <rtac_simulation/ReductionKernel.h>

namespace rtac { namespace simulation {

template <typename T>
__global__ void do_render_kernel(ImageView<T> out, KernelView2D<T> kernel, float xSpan, float ySpan)
{
    float x = xSpan*((((float)threadIdx.x) / (blockDim.x - 1)) - 0.5f); 
    float y = ySpan*((((float)blockIdx.x)  / (gridDim.x  - 1)) - 0.5f); 

    out(blockIdx.x, threadIdx.x) = kernel(x, y);
}


Image<float,cuda::DeviceVector> render_kernel(const Kernel2D<float>& kernel)
{
    Image<float,cuda::DeviceVector> res(kernel.width(), kernel.height());

    do_render_kernel<<<kernel.height(), kernel.width()>>>(
        res.view(), kernel.view(), kernel.x_span(), kernel.y_span());
    cudaDeviceSynchronize();

    return res;
}

} //namespace simulation
} //namespace rtac


