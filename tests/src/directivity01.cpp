#include <iostream>
using namespace std;

#include <rtac_base/cuda/texture_utils.h>
#include <rtac_base/cuda/vector_utils.h>
using namespace rtac::cuda;

#ifdef RTAC_SIMULATION_OPENGL_ENABLED
    #include <rtac_display/Display.h>
    #include <rtac_display/renderers/ImageRenderer.h>
    using namespace rtac::display;
#endif //RTAC_SIMULATION_OPENGL_ENABLED

#include <rtac_simulation/Directivity.h>
#include <rtac_simulation/factories/utilities.h>
using namespace rtac::simulation;

int main()
{
    auto directivity = Directivity::rectangle_antenna(0.1 / 64, 0.008, 1500 / 1.2e6,
                                                      "single-sided");
    //auto directivity = Directivity::disk_antenna(0.1, 1500.0 / 200.0e3, "single-sided");
    //auto directivity = Directivity::from_sinc_parameters(130.0f, 20.0f);
    //auto directivity = parse_directivity(
    //    YAML::LoadFile(FileFinder::Get()->find_one("EA400_200KHz_emitter.yaml"))["directivity"]);

    auto rendered = render_texture(directivity->texture());

    #ifdef RTAC_SIMULATION_OPENGL_ENABLED
    Display display;
    auto renderer = display.create_renderer<ImageRenderer>(View::Create());
    renderer->enable_colormap();

    renderer->texture()->set_image({rendered.width(),
                                    rendered.height()},
                                   GLVector<float>(rescale(rendered.container())));

    while(!display.should_close()) {
        display.draw();
    }
    #endif //RTAC_SIMULATION_OPENGL_ENABLED

    return 0;
}

