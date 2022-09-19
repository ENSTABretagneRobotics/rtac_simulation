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
#include <rtac_simulation/Directivity.h>
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
                                  DirectivityView directivity)
{
    for(auto w = threadIdx.x; w < out.width(); w += blockDim.x) {
        out(blockIdx.y, w) = abs(directivity(directions(blockIdx.y,w)));
    }
}

__global__ void render_emitter(rtac::types::ImageView<float> out, 
                               rtac::types::ImageView<const float3> directions,
                               EmitterView<float> emitter)
{
    for(auto w = threadIdx.x; w < out.width(); w += blockDim.x) {
        //float3 d = directions(blockIdx.y,w);
        float3 d = emitter.ray_direction(directions(blockIdx.y,w));
        out(blockIdx.y, w) = abs(emitter.sample_value(d));
        //out(blockIdx.y, w) = 0.5f*(emitter.sample_value(d).real() + 1.0f);
        //out(blockIdx.y, w) = emitter.sample_value(d).real();
    }
}

void render_emitter(GLVector<float>& dst, 
                    const rtac::types::Image<float3,DeviceVector>& directions,
                    Emitter<float>::ConstPtr emitter)
{
    dst.resize(directions.shape().area());
    {
        auto ptr = dst.map_cuda();
        render_emitter<<<{1,directions.height()},512>>>(
            rtac::types::ImageView<float>(directions.shape(),
                rtac::types::VectorView<float>(dst.size(), ptr)),
            directions.const_view(),
            emitter->view());
        cudaDeviceSynchronize();
        CUDA_CHECK_LAST();
    }
}

int main()
{
    //auto directivity = Directivity::from_sinc_parameters(130.0f, 20.0f);
    auto directivity = Directivity::from_sinc_parameters(20.0f, 20.0f);

    auto directions = generate_directions();
    auto emitter = Emitter<float>::Create(directions.container(), directivity);

    Display display;
    float fps = 60.0;
    display.limit_frame_rate(fps);
    display.enable_frame_counter();
    
    GLVector<float> data(directions.shape().area());
    {
        auto ptr = data.map_cuda();
        //render_directions<<<{1,directions.height()},512>>>(
        //    rtac::types::ImageView<float>(directions.shape(),
        //        rtac::types::VectorView<float>(data.size(), ptr)),
        //    directions.const_view(),
        //    directivity->view());
        render_emitter<<<{1,directions.height()},512>>>(
            rtac::types::ImageView<float>(directions.shape(),
                rtac::types::VectorView<float>(data.size(), ptr)),
            directions.const_view(),
            emitter->view());
        cudaDeviceSynchronize();
        CUDA_CHECK_LAST();
    }
    
    auto renderer = display.create_renderer<ImageRenderer>(View::New());
    renderer->enable_colormap();
    renderer->texture()->set_image(directions.shape(), data);
    
    auto r = rtac::simulation::Pose::from_rotation_matrix(
        Eigen::AngleAxisf(0.5f / fps, Eigen::Vector3f::UnitY()).toRotationMatrix());

    while(!display.should_close()) {
        emitter->pose() = emitter->pose() * r;
        render_emitter(data, directions, emitter);
        renderer->texture()->set_image(directions.shape(), data);
        display.draw();
    }
    return 0;
}




