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
    static const std::string vertexShader2;
    static const std::string fragmentShader2;

    protected:

    display::GLMesh::Ptr mesh_;

    RayCasterGL(const display::GLContext::Ptr& context);

    public:

    static Ptr Create(const display::GLContext::Ptr& context);

    display::GLMesh::ConstPtr   mesh() const { return mesh_; }
    const display::GLMesh::Ptr& mesh()       { return mesh_; }

    template <class MeshT>
    void set_mesh(const MeshT& mesh);

    virtual void draw(const display::View::ConstPtr& view) const;
    virtual void draw2(const display::View::ConstPtr& view) const;
};

template <class MeshT>
void RayCasterGL::set_mesh(const MeshT& mesh)
{
    if(!mesh_)
        mesh_ = display::GLMesh::Create();
    
    mesh_->points().set_data(mesh.points().size(),
       (const display::GLMesh::Point*)mesh.points().data());
    mesh_->faces().set_data(mesh.faces().size(),
       (const display::GLMesh::Face*)mesh.faces().data());
    mesh_->normals().set_data(mesh.normals().size(),
       (const display::GLMesh::Normal*)mesh.normals().data());
    mesh_->uvs().set_data(mesh.uvs().size(),
       (const display::GLMesh::UV*)mesh.uvs().data());
}

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_RAY_CASTER_GL_H_
