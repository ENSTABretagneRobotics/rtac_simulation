#include <iostream>
using namespace std;

#include <rtac_base/types/VectorView.h>
#include <rtac_base/types/Image.h>
using namespace rtac::types;

#include <rtac_base/cuda/texture_utils.h>

#include <rtac_display/Display.h>
#include <rtac_display/GLVector.h>
#include <rtac_display/renderers/ImageRenderer.h>
using namespace rtac::display;

#include <rtac_simulation/common.h>
#include <rtac_simulation/DirectivityFunction.h>
#include <rtac_simulation/Emitter.h>
using namespace rtac::simulation;

rtac::types::Image<float3, DeviceVector> generate_directions()
{
    unsigned int W = 512, H = 512;
    rtac::types::Image<float3, HostVector> dirs({W,H});
    for(int h = 0; h < H; h++) { 
        float phi = 0.5*M_PI * (2.0f*h / H - 1.0f);
        for(int w = 0; w < W; w++) {
            float theta = M_PI * (2.0f*w / W - 1.0f);
            dirs(h,w) = float3{cos(theta)*cos(phi),
                               sin(theta)*cos(phi),
                               sin(phi)};
        }
    }

    return dirs;
}

__global__ void render_directions(rtac::types::ImageView<float> out, 
                                  rtac::types::ImageView<const float3> directions,
                                  DirectivityView<float> directivity)
{
    for(auto w = threadIdx.x; w < out.width(); w += blockDim.x) {
        out(blockIdx.y, w) = abs(directivity(directions(blockIdx.y,w)));
    }
}

int main()
{
    auto directivity = Directivity<float>::from_sinc_parameters(130.0f * M_PIf / 180.0f, 
                                                                 20.0f * M_PIf / 180.0f);
    Display display;
    
    auto directions = generate_directions();
    GLVector<float> data(directions.shape().area());
    {
        auto ptr = data.map_cuda();
        render_directions<<<{1,directions.height()},512>>>(
            rtac::types::ImageView<float>(directions.shape(),
                rtac::types::VectorView<float>(data.size(), ptr)),
            directions.const_view(),
            directivity->view());
        cudaDeviceSynchronize();
        CUDA_CHECK_LAST();
    }
    
    auto renderer = display.create_renderer<ImageRenderer>(View::New());
    renderer->enable_colormap();
    renderer->texture()->set_image(directions.shape(), data);

    while(!display.should_close()) {
        display.draw();
    }
    return 0;
}




