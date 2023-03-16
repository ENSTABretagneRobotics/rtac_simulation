#include <rtac_simulation/EmitterGL.h>

namespace rtac { namespace simulation {

EmitterGL::EmitterGL(const Shape& outputSize, float fovy, const Pose& pose) :
    EmitterBase(pose),
    outputShape_(outputSize),
    view_(display::PinholeView::Create(fovy, pose))
{}

EmitterGL::Ptr EmitterGL::Create(float resolution,
                                 float bearingAperture,
                                 float elevationAperture,
                                 const Pose& pose)
{
    unsigned int width  = 2*(unsigned int)(0.5f*bearingAperture / resolution);
    unsigned int height = 2*(unsigned int)(0.5f*width*elevationAperture / bearingAperture);

    return Ptr(new EmitterGL({width, height}, elevationAperture, pose));
}

} //namespace simulation
} //namespace rtac

std::ostream& operator<<(std::ostream& os, const rtac::simulation::EmitterGL& emitter)
{
    os << "EmitterGL :"
       << "\n- pose : " << emitter.pose()
       << "\n- shape : " << emitter.width() << 'x' << emitter.height()
       << " (" << emitter.ray_count() << " rays)"
       << "\n vertical aperture : " << emitter.view().fovy()
       << "\n horizontal aperture : " << (emitter.view().fovy()*emitter.width()) / emitter.height();
    return os;
}
