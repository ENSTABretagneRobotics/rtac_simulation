#include <rtac_simulation/EmitterGL.h>

namespace rtac { namespace simulation {

EmitterGL::EmitterGL(const Shape& outputSize, float fovy, 
                     float frequency, const Pose& pose) :
    EmitterBase(frequency, pose),
    outputShape_(outputSize),
    toGLFrame_(rtac::Pose<float>::from_rotation_matrix(                      
        Eigen::AngleAxisf(-0.5f*M_PI, Eigen::Vector3f::UnitZ()).toRotationMatrix())),
    view_(display::PinholeView::Create(fovy, pose))
{}

EmitterGL::Ptr EmitterGL::Create(float resolution,
                                 float bearingAperture,
                                 float elevationAperture,
                                 float frequency, const Pose& pose)
{
    unsigned int width  = 2*(unsigned int)(0.5f*bearingAperture / resolution);
    unsigned int height = 2*(unsigned int)(0.5f*width*elevationAperture / bearingAperture);

    return Ptr(new EmitterGL({width, height}, elevationAperture, frequency, pose));
}

display::PinholeView::Ptr EmitterGL::view()
{
    auto oculusToGL = rtac::Pose<float>::from_rotation_matrix(                      
        Eigen::AngleAxisf(-0.5f*M_PI, Eigen::Vector3f::UnitZ()).toRotationMatrix());
    view_->set_pose(this->pose() * oculusToGL);
    return view_;
}

display::PinholeView::ConstPtr EmitterGL::view() const
{
    auto oculusToGL = rtac::Pose<float>::from_rotation_matrix(                      
        Eigen::AngleAxisf(-0.5f*M_PI, Eigen::Vector3f::UnitZ()).toRotationMatrix());
    view_->set_pose(this->pose() * oculusToGL);
    return view_;
}

} //namespace simulation
} //namespace rtac

std::ostream& operator<<(std::ostream& os, const rtac::simulation::EmitterGL& emitter)
{
    os << "EmitterGL :"
       << "\n- pose : " << emitter.pose()
       << "\n- shape : " << emitter.width() << 'x' << emitter.height()
       << " (" << emitter.ray_count() << " rays)"
       << "\n vertical aperture : " << emitter.view()->fovy()
       << "\n horizontal aperture : " << (emitter.view()->fovy()*emitter.width()) / emitter.height();
    return os;
}


