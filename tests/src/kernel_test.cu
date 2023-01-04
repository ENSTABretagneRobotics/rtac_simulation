#include <iostream>
using namespace std;

#include <rtac_base/containers/VectorView.h>
#include <rtac_base/containers/Image.h>
using namespace rtac;

#include <rtac_base/cuda/texture_utils.h>

#include <rtac_display/Display.h>
#include <rtac_display/GLVector.h>
#include <rtac_display/renderers/ImageRenderer.h>
using namespace rtac::display;

#include <rtac_simulation/common.h>
#include <rtac_simulation/helpers/receiver_factories.h>
#include <rtac_simulation/PolarKernel2D.h>
using namespace rtac::simulation;

__global__ void render_kernel(rtac::ImageView<float> out, 
                              PolarKernelView2D<float> kernel,
                              float bearingSpan, float rangeSpan)
{
    float range = rangeSpan * (((float)blockIdx.x) / (out.height() - 1) - 0.5f);
    for(auto w = threadIdx.x; w < out.width(); w += blockDim.x) {
        float bearing = bearingSpan * (((float)w) / (out.width() - 1) - 0.5f);

        float value = kernel(range, bearing);
        value = 0.5f * (value + 1.0f);
        //value = value * value;

        out(blockIdx.x, w) = value;
    }
}

int main()
{
    float bearingResolution = 0.6f;
    float pulseLength       = 0.04f;
    float wavelength        = 1500.0 / 1.2e6;

    //auto kernel = simple_polar_kernel<float>(bearingResolution,
    //                                         pulseLength, wavelength);
    auto kernel = simple_polar_kernel<float>(bearingResolution,
                                             //2*M_PI*bearingResolution,
                                             130.0f,
                                             pulseLength, wavelength,8);
    rtac::display::Shape shape({kernel->texture().width(),
                                kernel->texture().height()});
    cout << "Kernel shape : " << shape << endl;

    Display display;
    float fps = 60.0;
    display.limit_frame_rate(fps);
    display.enable_frame_counter();
    
    GLVector<float> data(shape.area());
    {
        auto ptr = data.map_cuda();
        render_kernel<<<shape.height, 512>>>(
            rtac::ImageView<float>(shape,
                rtac::VectorView<float>(data.size(), ptr)),
            kernel->view(),
            kernel->bearing_span(), 
            kernel->range_span());
        cudaDeviceSynchronize();
        CUDA_CHECK_LAST();
    }
    
    auto renderer = display.create_renderer<ImageRenderer>(View::Create());
    renderer->enable_colormap();
    renderer->texture()->set_image(shape, data);

    while(!display.should_close()) {
        display.draw();
    }
    return 0;
}




