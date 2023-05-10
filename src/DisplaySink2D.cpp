#include <rtac_simulation/DisplaySink2D.h>

namespace rtac { namespace simulation {

DisplaySink2D::DisplaySink2D(const display::GLFWContext::Ptr& glContext,
                             const std::string& title) :
    Sink2D(title),
    window_(800, 600, title, glContext),
    renderer_(window_.create_renderer<display::FanRenderer>(display::View::Create()))
{}

DisplaySink2D::Ptr DisplaySink2D::Create(const std::string& title)
{
    // glContext will be created automatically
    return Ptr(new DisplaySink2D(nullptr, title));
}

DisplaySink2D::Ptr DisplaySink2D::Create(const display::GLFWContext::Ptr& glContext,
                                         const std::string& title)
{
    return Ptr(new DisplaySink2D(glContext, title));
}

void DisplaySink2D::set_output(const cuda::CudaPing2D<float>& ping)
{
    renderer_->set_ping(ping);
    window_.draw();
}

void DisplaySink2D::set_output(const cuda::CudaPing2D<Complex<float>>& ping)
{
    renderer_->set_ping(ping);
    window_.draw();
}

} //namespace simulation
} //namespace rtac
