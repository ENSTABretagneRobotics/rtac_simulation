#ifndef _DEF_RTAC_SIMULATION_EMITTER_GL_H_
#define _DEF_RTAC_SIMULATION_EMITTER_GL_H_

#include <memory>

#include <rtac_display/views/PinholeView.h>

#include <rtac_simulation/Emitter.h>

namespace rtac { namespace simulation {

class EmitterGL : public EmitterBase
{
    public:

    using Ptr      = std::shared_ptr<EmitterGL>;
    using ConstPtr = std::shared_ptr<const EmitterGL>;

    using Shape = display::PinholeView::Shape;

    protected:
    
    Shape outputShape_;

    display::PinholeView::Ptr view_;

    EmitterGL(const Shape& outputSize, float fovy, //fovy = elevation aperture
              const Pose& pose);

    public:

    static Ptr Create(float resolution,
                      float bearingAperture,
                      float elevationAperture,
                      const Pose& pose = Pose());

          display::PinholeView& view()       { return *view_; }
    const display::PinholeView& view() const { return *view_; }

    const Shape& output_shape() const { return outputShape_;        }
    unsigned int ray_count()    const { return outputShape_.area(); }
    unsigned int width()        const { return outputShape_.width;  }
    unsigned int height()       const { return outputShape_.height; }
};

} //namespace simulation
} //namespace rtac

std::ostream& operator<<(std::ostream& os, const rtac::simulation::EmitterGL& emitter);

#endif //_DEF_RTAC_SIMULATION_EMITTER_GL_H_
