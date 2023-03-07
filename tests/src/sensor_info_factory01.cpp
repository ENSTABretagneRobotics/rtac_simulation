#include <iostream>
using namespace std;

#include <rtac_base/containers/Image.h>
using namespace rtac;

#include <rtac_base/cuda/DeviceVector.h>
#include <rtac_base/cuda/texture_utils.h>
#include <rtac_base/cuda/vector_utils.h>
using namespace rtac::cuda;

#include <rtac_display/Display.h>
#include <rtac_display/renderers/ImageRenderer.h>
using namespace rtac::display;

#include <rtac_simulation/factories/utilities.h>
#include <rtac_simulation/factories/SensorInfoFactory.h>
using namespace rtac::simulation;

int main()
{
    auto finder = FileFinder::Get({std::string(RTAC_TEST_CONFIG)});
    auto filename = finder->find_one(".*oculus_M1200d_1.yaml");
    std::cout << "config file : " << filename << std::endl;

    auto sensorInfo = SensorInfoFactory2D::Make(YAML::LoadFile(filename));

    auto psf = sensorInfo->point_spread_function();
    Image<float2, DeviceVector> tmp(psf->width(), psf->height());
    render_texture(complex_cast(psf)->texture(), tmp.view());
    Display displayPsf;
    auto renderer = displayPsf.create_renderer<ImageRenderer>(View::Create());
    renderer->texture()->set_image({psf->width(), psf->height()},
                                    GLVector<float2>(tmp.container()));

    auto directivity = sensorInfo->directivity();
    Image<float, DeviceVector> tmp0(directivity->texture().width(), 
                                    directivity->texture().height());
    render_texture(directivity->texture(), tmp0.view());
    Display displayDir;
    auto renderer0 = displayDir.create_renderer<ImageRenderer>(View::Create());
    renderer0->enable_colormap();

    renderer0->texture()->set_image({tmp0.width(), tmp0.height()},
                                    GLVector<float>(rescale(tmp0.container())));

    while(!displayPsf.should_close() &&
          !displayDir.should_close())
    {
        displayPsf.draw();
        displayDir.draw();
    }

    return 0;
}



