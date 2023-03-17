#ifndef _DEF_RTAC_SIMULATION_RAY_CASTER_GL_H_
#define _DEF_RTAC_SIMULATION_RAY_CASTER_GL_H_

#include <memory>

#include <rtac_display/GLMesh.h>
#include <rtac_display/views/View3D.h>
#include <rtac_display/renderers/Renderer.h>

namespace rtac { namespace simulation {

class RayCasterGL : rtac::display::Renderer
{
    public:

    using Ptr      = std::shared_ptr<RayCasterGL>;
    using ConstPtr = std::shared_ptr<const RayCasterGL>;

    static const std::string vertexShader;
    static const std::string fragmentShader;

    protected:

    display::GLMesh::ConstPtr mesh_;

    RayCasterGL(const display::GLContext::Ptr& context);

    public:

    static Ptr Create(const display::GLContext::Ptr& context);

    display::GLMesh::ConstPtr  mesh() const { return mesh_; }
    display::GLMesh::ConstPtr& mesh()       { return mesh_; }

    template <class MeshT>
    void set_mesh(const MeshT& mesh);

    virtual void draw(const display::View::ConstPtr& view) const;
};

template <class MeshT>
void RayCasterGL::set_mesh(const MeshT& mesh)
{
    if(!mesh_)
        mesh_ = display::GLMesh::Create();
    mesh_ = mesh;    
}

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_RAY_CASTER_GL_H_
