#ifndef _DEF_RTAC_SIMULATION_SIMULATION_GL_H_
#define _DEF_RTAC_SIMULATION_SIMULATION_GL_H_

#include <memory>

#include <rtac_display/Display.h>
#include <rtac_display/GLFrameBuffer.h>
#include <rtac_display/GLRenderBuffer.h>

#include <rtac_simulation/Simulation.h>
#include <rtac_simulation/EmitterGL.h>

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

    display::GLFrameBuffer::Ptr  frameBuffer_;
    display::GLRenderBuffer::Ptr renderTarget_;
    display::GLRenderBuffer::Ptr depthBuffer_;

    SimulationGL(const EmitterGL::Ptr& emitter,
                 const SensorInstance::Ptr& receiver);

    //EmitterGL::Ptr&      emitter_ptr()  { return emitter_;  }
    //SensorInstance::Ptr& receiver_ptr() { return receiver_; }

    public:

    static Ptr Create(const EmitterGL::Ptr& emitter,
                      const SensorInstance::Ptr& receiver);
    static Ptr Create(const EmitterBase::Ptr& emitter,
                      const SensorInstance::Ptr& receiver);
    static Ptr Create(const std::string& emitterFilename,
                      const std::string& receiverFilename);

    const EmitterGL&      emitter()  const { return *emitter_;  }
          EmitterGL&      emitter()        { return *emitter_;  }
    const SensorInstance& receiver() const { return *receiver_; }
          SensorInstance& receiver()       { return *receiver_; }
    void run() {}
};

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_SIMULATION_GL_H_
