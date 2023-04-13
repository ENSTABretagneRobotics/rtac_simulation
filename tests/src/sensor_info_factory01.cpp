#include <iostream>
using namespace std;

#include <rtac_base/containers/Image.h>
using namespace rtac;

#include <rtac_base/cuda/CudaVector.h>
#include <rtac_base/cuda/texture_utils.h>
#include <rtac_base/cuda/vector_utils.h>
using namespace rtac::cuda;

#ifdef RTAC_SIMULATION_OPENGL_ENABLED
    #include <rtac_display/Display.h>
    #include <rtac_display/renderers/ImageRenderer.h>
    using namespace rtac::display;
#endif //RTAC_SIMULATION_OPENGL_ENABLED

#include <rtac_simulation/factories/utilities.h>
#include <rtac_simulation/factories/SensorInfoFactory.h>
using namespace rtac::simulation;

int main()
{
    auto finder = FileFinder::Get();
    auto filename = finder->find_one(".*oculus_M1200d_1_receiver.yaml");
    std::cout << "config file oculus : " << filename << std::endl;
    auto sensorInfo = SensorInfoFactory::Make(YAML::LoadFile(filename));

    auto filename2 = finder->find_one("EA400_200KHz_receiver.yaml");
    std::cout << "config file ea400 : " << filename2 << std::endl;
    auto sensorInfo2 = SensorInfoFactory::Make(YAML::LoadFile(filename2));

    auto directivity = sensorInfo->directivity();
    Image<float, CudaVector> tmp0(directivity->texture().width(), 
                                  directivity->texture().height());
    render_texture(directivity->texture(), tmp0.view());

    #ifdef RTAC_SIMULATION_OPENGL_ENABLED
    Display displayDir;
    auto renderer0 = displayDir.create_renderer<ImageRenderer>(View::Create());
    renderer0->enable_colormap();

    renderer0->texture()->set_image({tmp0.width(), tmp0.height()},
                                    GLVector<float>(rescale(tmp0.container())));

    while(!displayDir.should_close())
    {
        displayDir.draw();
    }
    #endif //RTAC_SIMULATION_OPENGL_ENABLED

    return 0;
}



