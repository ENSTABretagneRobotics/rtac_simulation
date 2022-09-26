#include "EmitterGL.h"

namespace rtac { namespace display {

const std::string EmitterGL::vertexShader = std::string(R"(
#version 430 core

in vec3 point;
in vec3 normal;

uniform mat4 view;
uniform vec3 origin;

out vec4 datum;

//out float range;
//out float a;

void main()
{
    gl_Position = view*vec4(point, 1.0f);

    //vec3 delta = point - origin;
    //float rangeSquared = dot(delta, delta);
    //
    //a = abs(normalize((view*vec4(normal, 0.0f)).xyz).z) / rangeSquared;
    //range = sqrt(rangeSquared);

    float a = abs(normalize((view*vec4(normal, 0.0f)).xyz).z);
    datum = vec4(a,a,a,1.0f);
}
)");

const std::string EmitterGL::fragmentShader = std::string(R"(
#version 430 core

in  vec4 datum;

//#define M_PI             3.1415926538
//#define reflectionShift  0.5*M_PI;
//#define wavelengthFactor 4.0f*M_PI / (1500.0 / 1.2e6f)
//
//in float range;
//in float a;

out vec4 outColor;

void main()
{
    outColor = datum;

    //float phase = range*wavelengthFactor + reflectionShift;
    //outColor = vec4(a*cos(phase),
    //                a*sin(phase),
    //                range,
    //                0.0f);
}

)");

EmitterGL::EmitterGL(const GLContext::Ptr& context) :
    Renderer(context, vertexShader, fragmentShader),
    mesh_(GLMesh::Create())
{}

EmitterGL::Ptr EmitterGL::Create(const GLContext::Ptr& context)
{
    return Ptr(new EmitterGL(context));
}

void EmitterGL::draw(const View::ConstPtr& view) const
{
    if(mesh_->points().size() == 0 ||
       mesh_->normals().size() != mesh_->points().size()) {
        std::cerr << "EmitterGL : cannot draw" << std::endl;
        std::cerr << *mesh_ << std::endl;
        return;
    }

    glUseProgram(this->renderProgram_);

    mesh_->points().bind(GL_ARRAY_BUFFER);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL);
    glEnableVertexAttribArray(0);

    mesh_->normals().bind(GL_ARRAY_BUFFER);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, NULL);
    glEnableVertexAttribArray(1);

    View3D::Mat4 viewMatrix = view->view_matrix();
    glUniformMatrix4fv(glGetUniformLocation(this->renderProgram_, "view"),
        1, GL_FALSE, viewMatrix.data());

    if(mesh_->faces().size() == 0) {
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        glDrawArrays(GL_TRIANGLES, 0, mesh_->points().size());
    }
    else {
        mesh_->faces().bind(GL_ELEMENT_ARRAY_BUFFER);
        glDrawElements(GL_TRIANGLES, 3*mesh_->faces().size(), GL_UNSIGNED_INT, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }

    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glUseProgram(0);

    GL_CHECK_LAST();
}

} //namespace display
} //namespace rtac
