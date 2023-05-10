#ifndef _DEF_RTAC_SIMULATION_DISPLAY_SINK_H_
#define _DEF_RTAC_SIMULATION_DISPLAY_SINK_H_

#include <memory>

#include <rtac_display/Display.h>
#include <rtac_display/renderers/FanRenderer.h>

#include <rtac_simulation/sinks/Sink.h>

namespace rtac { namespace simulation {

class DisplaySink : public Sink
{
    public:

    using Ptr      = std::shared_ptr<DisplaySink>;
    using ConstPtr = std::shared_ptr<const DisplaySink>;

    protected:

    display::Display window_;
    display::FanRenderer::Ptr renderer_;

    DisplaySink(const display::GLFWContext::Ptr& glContext,
                const std::string& title);

    public:

    static Ptr Create(const std::string& title = "rtac_sim_2d");
    static Ptr Create(const display::GLFWContext::Ptr& glContext,
                      const std::string& title = "rtac_sim_2d");

    const display::Display& window() const { return window_; }
          display::Display& window()       { return window_; }
    
    const display::FanRenderer& renderer() const { return *renderer_; }
          display::FanRenderer& renderer()       { return *renderer_; }

    virtual void set_output(const SensorInstance::Ptr& sensor) override;
};

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_DISPLAY_SINK_H_
