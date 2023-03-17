#ifndef _DEF_RTAC_SIMULATION_SIMULATION_GL_H_
#define _DEF_RTAC_SIMULATION_SIMULATION_GL_H_

#include <memory>

#include <rtac_display/Display.h>
#include <rtac_display/GLVector.h>
#include <rtac_display/GLFrameBuffer.h>
#include <rtac_display/GLRenderBuffer.h>

#include <rtac_simulation/Simulation.h>
#include <rtac_simulation/EmitterGL.h>
#include <rtac_simulation/RayCasterGL.h>

namespace rtac { namespace simulation {

class SimulationGL : public Simulation1
{
    public:

    using Ptr      = std::shared_ptr<SimulationGL>;
    using ConstPtr = std::shared_ptr<const SimulationGL>;

    protected:

    EmitterGL::Ptr      emitter_;
    SensorInstance::Ptr receiver_;

    std::shared_ptr<display::Display> drawSurface_;

    display::GLFrameBuffer::Ptr    frameBuffer_;
    display::GLRenderBuffer::Ptr   renderTarget_;
    display::GLRenderBuffer::Ptr   depthBuffer_;
    RayCasterGL::Ptr               rayCaster_;
    display::GLVector<SimSample2D> casterOutput_;

    SimulationGL(const EmitterGL::Ptr& emitter,
                 const SensorInstance::Ptr& receiver);

    void fill_receiver();

    //EmitterGL::Ptr&      emitter_ptr()  { return emitter_;  }
    //SensorInstance::Ptr& receiver_ptr() { return receiver_; }

    public:

    static Ptr Create(const EmitterGL::Ptr& emitter,
                      const SensorInstance::Ptr& receiver);
    static Ptr Create(const EmitterBase::Ptr& emitter,
                      const SensorInstance::Ptr& receiver);
    static Ptr Create(const std::string& emitterFilename,
                      const std::string& receiverFilename);

    display::GLFWContext::Ptr context()       { return drawSurface_->context(); }
    display::GLFWContext::Ptr context() const { return drawSurface_->context(); }

    const RayCasterGL::Ptr& ray_caster()       { return rayCaster_; }
    RayCasterGL::ConstPtr   ray_caster() const { return rayCaster_; }

    const display::GLVector<SimSample2D>& caster_output() const { return casterOutput_; }

    const EmitterGL&      emitter()  const { return *emitter_;  }
          EmitterGL&      emitter()        { return *emitter_;  }
    const SensorInstance& receiver() const { return *receiver_; }
          SensorInstance& receiver()       { return *receiver_; }
    const SensorInstance::Ptr& receiver_ptr()       { return receiver_; }
    SensorInstance::ConstPtr   receiver_ptr() const { return receiver_; }

    void run();
};

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_SIMULATION_GL_H_
