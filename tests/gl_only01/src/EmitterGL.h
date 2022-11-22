#ifndef _DEF_RTAC_SIMULATION_EMITTER_GL_H_
#define _DEF_RTAC_SIMULATION_EMITTER_GL_H_

#include <rtac_base/types/common.h>

#include <rtac_display/GLContext.h>
#include <rtac_display/renderers/Renderer.h>
#include <rtac_display/views/View3D.h>
#include <rtac_display/GLMesh.h>

// using display namespace for convenience. Will go in simulation namespace
// once stable enough.
namespace rtac { namespace display {

class EmitterGL : public Renderer
{
    public:

    using Ptr      = rtac::Handle<EmitterGL>;
    using ConstPtr = rtac::Handle<const EmitterGL>;

    using Mat4 = View3D::Mat4;
    using Pose = View3D::Pose;

    protected:

    static const std::string vertexShader;
    static const std::string fragmentShader;

    GLMesh::ConstPtr mesh_;
    Pose             pose_; // not used (for now)

    EmitterGL(const GLContext::Ptr& context);

    public:

    static Ptr Create(const GLContext::Ptr& context);

    GLMesh::ConstPtr  mesh() const { return mesh_; }
    GLMesh::ConstPtr& mesh()       { return mesh_; }

    virtual void draw(const View::ConstPtr& view) const;
};

} //namespace display
} //namespace rtac


#endif //_DEF_RTAC_SIMULATION_EMITTER_GL_H_
