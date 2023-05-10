#include <rtac_simulation/sinks/DisplaySink.h>

namespace rtac { namespace simulation {

DisplaySink::DisplaySink(const display::GLFWContext::Ptr& glContext,
                         const std::string& title) :
    Sink(title),
    window_(800, 600, title, glContext),
    renderer_(window_.create_renderer<display::FanRenderer>(display::View::Create()))
{}

DisplaySink::Ptr DisplaySink::Create(const std::string& title)
{
    // glContext will be created automatically
    return Ptr(new DisplaySink(nullptr, title));
}

DisplaySink::Ptr DisplaySink::Create(const display::GLFWContext::Ptr& glContext,
                                     const std::string& title)
{
    return Ptr(new DisplaySink(glContext, title));
}

void DisplaySink::set_output(const SensorInstance::Ptr& sensor)
{
    if(!sensor->is_type<SensorInstance>()) {
        std::cerr << "FileSink : sensor type not handled" << std::endl;
        return;
    }
    switch(sensor->scalar_type()) {
        default: throw TypeError() << " : only float and Complex<float> are supported";
        case RTAC_FLOAT:
            renderer_->set_ping(
                      sensor->safe_cast<SensorInstance2D_2<float>>()->get_ping());
            break;
        case RTAC_CFLOAT:
            renderer_->set_ping(
                      sensor->safe_cast<SensorInstance2D_2<Complex<float>>>()->get_ping());
            break;
    }
    window_.draw();
}


} //namespace simulation
} //namespace rtac
