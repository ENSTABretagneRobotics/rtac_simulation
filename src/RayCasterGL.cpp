#include <rtac_simulation/RayCasterGL.h>

namespace rtac { namespace simulation {

const std::string RayCasterGL::vertexShader = std::string(R"(
#version 430 core

in vec3 point;
in vec3 normal;

uniform mat4 worldToLocal;
uniform mat4 worldToScreen;

uniform vec3 origin;

out vec3  delta;
out float reflected;

void main()
{
    gl_Position = worldToScreen*vec4(point, 1.0f);

    delta     = (worldToLocal*vec4(point - origin, 0.0f)).xyz;
    reflected = abs(normalize((worldToLocal*vec4(normal, 0.0f)).xyz).z);
}
)");

const std::string RayCasterGL::fragmentShader = std::string(R"(
#version 430 core

#define M_PI             3.1415926538
#define iM_PI            1.0 / M_PI
#define reflectionShift  0.5*M_PI;
#define wavelengthFactor 4.0f*M_PI / (1500.0 / 1.2e6f)

in vec3  delta;
in float reflected;

//uniform sampler2D directivity;

out vec4 outColor;

void main()
{
    float squaredRange = dot(delta, delta);
    float a            = reflected / squaredRange;
    float range        = sqrt(squaredRange);
    float bearing      = atan(delta.x, -delta.z);
    float phase        = range*wavelengthFactor + reflectionShift;

    float elevation    = atan(delta.y, -delta.z);
    //a *= texture(directivity, vec2(iM_PI*bearing, iM_PI*elevation)).x;
    a /= cos(bearing);
    a /= cos(elevation);
    
    outColor = vec4(a*cos(phase),
                    a*sin(phase),
                    range,
                    bearing);
}

)");

RayCasterGL::RayCasterGL(const display::GLContext::Ptr& context) :
    rtac::display::Renderer(context, vertexShader, fragmentShader),
    soundCelerity_(1500.0f)
{}

RayCasterGL::Ptr RayCasterGL::Create(const display::GLContext::Ptr& context)
{
    return Ptr(new RayCasterGL(context));
}

void RayCasterGL::draw2(const display::View::ConstPtr&) const
{
    float points[] = {-1,-1, 0, 1,
                       1,-1, 0, 1,
                       1, 1, 0, 1,
                      -1,-1, 0, 1,
                       1, 1, 0, 1,
                      -1, 1, 0, 1};

    glUseProgram(this->renderProgram_);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, points);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glDrawArrays(GL_TRIANGLES, 0, 6);

    glDisableVertexAttribArray(0);

    GL_CHECK_LAST();
}

void RayCasterGL::draw(const display::View::ConstPtr& view) const
{
    using namespace rtac::display;
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
    glUniformMatrix4fv(glGetUniformLocation(this->renderProgram_, "worldToScreen"),
        1, GL_FALSE, viewMatrix.data());

    auto view3d = std::dynamic_pointer_cast<const View3D>(view);
    if(!view3d) {
        throw std::runtime_error("EmitterGL must be used with a View3D instance or derivatives.");
    }
    View3D::Mat4 worldToLocal = view3d->raw_view_matrix().inverse();
    glUniformMatrix4fv(glGetUniformLocation(this->renderProgram_, "worldToLocal"),
        1, GL_FALSE, worldToLocal.data());

    View3D::Vec3 origin = view3d->translation();
    glUniform3f(glGetUniformLocation(this->renderProgram_, "origin"),
        origin(0), origin(1), origin(2));

    //glUniform1i(glGetUniformLocation(this->renderProgram_, "directivity"), 0);
    //glActiveTexture(GL_TEXTURE0);
    //directivity_.bind(GL_TEXTURE_2D);

    if(mesh_->faces().size() == 0) {
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        glDrawArrays(GL_TRIANGLES, 0, mesh_->points().size());
    }
    else {
        mesh_->faces().bind(GL_ELEMENT_ARRAY_BUFFER);
        glDrawElements(GL_TRIANGLES, 3*mesh_->faces().size(), GL_UNSIGNED_INT, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }

    glBindTexture(GL_TEXTURE_2D, 0);

    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glUseProgram(0);

    GL_CHECK_LAST();
}

} //namespace simulation
} //namespace rtac
