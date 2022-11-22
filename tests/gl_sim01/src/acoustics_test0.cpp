#include <iostream>
#include <thread>
using namespace std;

#include <rtac_simulation/helpers/OculusRosbagIterator.h>

#include <narval_oculus/Oculus.h>
#include <narval_oculus/helpers/OculusRenderer.h>

#include <rtac_base/files.h>
#include <rtac_base/external/obj_codec.h>
using namespace rtac::files;
#include <rtac_base/navigation.h>
using namespace rtac::navigation;
#include <rtac_base/types/Mesh.h>
#include <rtac_base/types/Pose.h>
using Mesh = rtac::Mesh<>;
using Pose = rtac::Pose<float>;

#include <rtac_base/types/Pose.h>
using Pose = rtac::Pose<float>;
using Vec3       = Pose::Vec3;
using Mat3       = Pose::Mat3;
using Mat4       = Pose::Mat4;
using Quaternion = Pose::Quat;
#include <rtac_base/interpolation.h>
using namespace rtac::algorithm;

#include <rtac_base/cuda/Texture2D.h>
#include <rtac_base/cuda/DeviceVector.h>
#include <rtac_base/cuda/DeviceObject.h>
#include <rtac_base/cuda/HostVector.h>
using namespace rtac::cuda;

#include <rtac_display/samples/Display3D.h>
#include <rtac_display/renderers/MeshRenderer.h>
#include <rtac_display/renderers/PointCloudRenderer.h>
#include <rtac_display/renderers/ImageRenderer.h>
#include <rtac_display/renderers/FrameInstances.h>
#include <rtac_display/renderers/Frame.h>
#include <rtac_display/GLFrameBuffer.h>
#include <rtac_display/GLRenderBuffer.h>
namespace plt = rtac::display;

#include <rtac_simulation/helpers/PolarTargetRenderer.h>

#include <rtac_optix/utils.h>
#include <rtac_optix/Handle.h>
#include <rtac_optix/Context.h>
#include <rtac_optix/Pipeline.h>
#include <rtac_optix/MeshGeometry.h>
#include <rtac_optix/ObjectInstance.h>
#include <rtac_optix/GroupInstance.h>
#include <rtac_optix/ShaderBindingTable.h>
#include <rtac_optix/ShaderBinding.h>
using namespace rtac::optix;

#include <rtac_simulation/examples/blueprint_oculus.h>

#include <gl_sim01_rtac_simulation/ptx_files.h>

#include "oculus_sim.h"
#include "EmitterGL.h"
//using namespace rtac::simulation;

MeshGeometry::Ptr geometry_from_mesh(const Context::Ptr& ctx, const Mesh& mesh);
DeviceVector<float> dtm_phases(const MeshGeometry::Ptr& mesh);

int main()
{
    auto dtmPath = find_one(".*models3d/pyramide2_test01");
    auto bagPath = find_one(".*pyramide2_matisse_positions.bag");
    //auto dtmPath = find_one(".*models3d/pyramide2_test01", "/media/pnarvor/Samsung_T5/submeeting/save_sabre01");
    //auto bagPath = find_one(".*pyramide2_matisse_positions.bag", "/media/pnarvor/Samsung_T5/submeeting/save_sabre01");

    cout << "DTM path : " << dtmPath << endl;

    auto Roculus = rtac::Pose<float>::from_rotation_matrix(
        Eigen::AngleAxisf(0.5f*M_PI, Eigen::Vector3f::UnitZ()).toRotationMatrix());
    rtac::simulation::OculusRosbagIterator bag(bagPath, 
                                               "/oculus_sonar/ping",
                                               "/matisse_pose",
                                               Roculus);

    rtac::external::ObjLoader parser(dtmPath);
    parser.load_geometry();

    auto mesh = parser.create_single_mesh<HostMesh<>>();
    cout << "Number of points : " << mesh->points().size() << endl;
    cout << "Number of faces  : " << mesh->faces().size() << endl;

    auto ptxFiles = gl_sim01_rtac_simulation::get_ptx_files();
    optix_init();

    auto raycaster = rtac::simulation::PolarRayCaster::Create();
    auto context   = raycaster->context();

    //auto context  = Context::Create();
    //auto pipeline = Pipeline::Create(context);
    //pipeline->compile_options().numPayloadValues = 8;
    //pipeline->compile_options().exceptionFlags = OPTIX_EXCEPTION_FLAG_DEBUG;
    //auto module   = pipeline->add_module("oculus_sonar", ptxFiles["src/oculus_sim.cu"]);
    //auto raygen   = pipeline->add_raygen_program("__raygen__oculus_sonar", "oculus_sonar");
    //auto miss     = pipeline->add_miss_program("__miss__oculus_sonar", "oculus_sonar");
    raycaster->pipeline()->add_module("oculus_sonar", ptxFiles["src/oculus_sim.cu"]);
    
    auto hitSonar = raycaster->pipeline()->add_hit_programs();
    
    DeviceVector<float> dtmPhases;
    ObjectInstance::Ptr dtmObject; {
        auto geom = MeshGeometry::Create(context, DeviceMesh<>::Create(*mesh));
        geom->material_hit_setup({OPTIX_GEOMETRY_FLAG_NONE});
        geom->enable_vertex_access();
        dtmPhases = dtm_phases(geom);

        dtmObject = ObjectInstance::Create(geom);

        hitSonar->set_closesthit({"__closesthit__oculus_sonar_phased",
                                  raycaster->pipeline()->module("oculus_sonar")});
        dtmObject->add_material(rtac::simulation::SonarMaterial::Create(
            hitSonar, rtac::simulation::PhaseData({dtmPhases.size(), dtmPhases.data()})));
    }

    //auto topObject = GroupInstance::Create(context);
    //topObject->add_instance(dtmObject);

    //auto sbt = ShaderBindingTable::Create(Raytypes::RaytypeCount);
    //sbt->set_raygen_record(ShaderBinding<void>::Create(raygen));
    //sbt->add_miss_record(SonarMissMaterial::Create(miss));
    //sbt->add_object(dtmObject);

    raycaster->object_tree()->add_instance(dtmObject);
    raycaster->sbt()->add_object(dtmObject);

    auto directions = rtac::simulation::Emitter<float>::generate_polar_directions(0.1f,
                                                                                  140.0f,
                                                                                  100.0f);
    auto oculusEmitter = rtac::simulation::Emitter<float>::Create(
        //directions.container(), 
        directions,
        rtac::simulation::Directivity::from_sinc_parameters(130.0f, 20.0f));
    auto oculusReceiver = rtac::simulation::OculusReceiver::Create();

    //DeviceObject<Params> optixParams;

    //optixParams.topObject = *topObject;
    //optixParams.directions = directions.data();

    DeviceVector<float3>  optixPoints;
    //optixPoints.resize(directions.size());
    //optixParams.outputPoints = optixPoints.data();

    plt::samples::Display3D display;
    display.disable_frame_counter();
    //display.limit_frame_rate(60.0);
    display.view()->set_range(0.1f, 10000.0f);
    //display.view()->look_at({0,0,0},{5,4,122});
    display.view()->look_at({0,0,0},{5,4,3});
    auto glMesh = plt::GLMesh::Create();
    //*glMesh = mesh;
    glMesh->points().set_data(mesh->points().size(), (const plt::GLMesh::Point*)mesh->points().data());
    glMesh->faces().set_data(mesh->faces().size(),   (const plt::GLMesh::Face*)mesh->faces().data());
    glMesh->compute_normals();

    cout << "GLMesh : " << *glMesh << endl;

    auto renderer = display.create_renderer<plt::MeshRenderer>(display.view());
    renderer->mesh() = glMesh;
    auto optixRenderer = display.create_renderer<plt::PointCloudRenderer>(display.view());
    optixRenderer->set_color({0.5,0.0,0.0,1.0});
    auto trace = display.create_renderer<plt::FrameInstances>(display.view());

    plt::Display sonarDisplay(800, 600, "Oculus data", display.context());
    sonarDisplay.disable_frame_counter();
    auto pingRenderer = sonarDisplay.create_renderer<plt::OculusRenderer>(plt::View::Create());

    plt::Display simDisplay(800, 600, "Optix simulation", display.context());
    simDisplay.disable_frame_counter();
    auto simRenderer = simDisplay.create_renderer<plt::PolarTargetRenderer>(plt::View::Create());


    plt::Display simDisplay2(800, 600, "OpenGL simulation", display.context());
    simDisplay2.disable_frame_counter();
    auto simRenderer2 = simDisplay2.create_renderer<plt::PolarTargetRenderer>(plt::View::Create());

    
    // insonification with OpenGL
    auto oculusToGL = rtac::simulation::Pose::from_rotation_matrix(
        Eigen::AngleAxisf(-0.5f*M_PI, Eigen::Vector3f::UnitZ()).toRotationMatrix());
    float bearingAperture   = 130.0f;
    float elevationAperture = 90.0f;
    float angularResolution = 0.1f;

    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
    plt::Display glSim((size_t)   bearingAperture / angularResolution,
                       (size_t) elevationAperture / angularResolution,
                       "opengl_insonification",
                       display.context());
    cout << "GLSim shape : " << glSim.window_shape() << endl;
    auto glSimView = plt::PinholeView::Create(100.0f);
    //glSimView->set_range(0.1f, 10000.0f);
    //auto renderer2 = glSim.create_renderer<plt::MeshRenderer>(glSimView);
    //auto renderer2 = glSim.create_renderer<plt::EmitterGL>(glSimView);
    auto fbRenderer = glSim.create_renderer<plt::ImageRenderer>(plt::View::Create());

    auto renderer2 = plt::EmitterGL::Create(display.context());
    renderer2->mesh() = glMesh;

    auto frameBuffer  = plt::GLFrameBuffer::Create();
    //auto renderTarget = plt::GLTexture::Create();
    //renderTarget->resize<rtac::Point4<float>>(glSim.window_shape());
    //fbRenderer->texture() = renderTarget;
    auto renderTarget = plt::GLRenderBuffer::Create(glSim.window_shape(), GL_RGBA32F);
    auto depthBuffer  = plt::GLRenderBuffer::Create(glSim.window_shape(), GL_DEPTH24_STENCIL8);

    frameBuffer->bind(GL_FRAMEBUFFER);
    //renderTarget->bind(GL_TEXTURE_2D);
    //glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, renderTarget->gl_id(), 0);
    renderTarget->bind();
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, renderTarget->gl_id());
    depthBuffer->bind();
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, depthBuffer->gl_id());
    if(!frameBuffer->is_complete()) {
        throw std::runtime_error("FBO not complete");
    }
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
    //glBindTexture(GL_TEXTURE_2D, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    GL_CHECK_LAST();

    plt::GLVector<rtac::simulation::PolarSample2D<float>> pixelOutput(glSim.window_shape().area());
    //plt::GLVector<float4> pixelOutput(glSim.window_shape().area());
    //plt::GLVector<rtac::Point4<float>> pixelOutput(glSim.window_shape().area());
    auto glReceiver = rtac::simulation::OculusReceiver::Create();
    
    int count = 0;
    int screenshotCount = 0;
    while(!display.should_close() &&
          !glSim.should_close() &&
          !simDisplay.should_close() &&
          !simDisplay2.should_close() &&
          !sonarDisplay.should_close())
    {
        auto oculusDatum = bag.next();

        auto meta     = oculusDatum.ping_metadata();
        auto pingData = oculusDatum.ping_data();
        auto pose     = oculusDatum.pose();

        oculusEmitter->pose()  = pose;
        oculusReceiver->pose() = pose;
        oculusReceiver->samples().resize(directions.size());
        oculusReceiver->reconfigure(meta, pingData);

        raycaster->trace(oculusEmitter, oculusReceiver,
                         directions, optixPoints);

        //optixParams.emitter    = oculusEmitter->view();
        //optixParams.receiver   = oculusReceiver->view();

        //optixParams.update_device();
        //CUDA_CHECK_LAST();

        //OPTIX_CHECK( optixLaunch(*pipeline, 0,
        //                 (CUdeviceptr)optixParams.device_ptr(), sizeof(Params),
        //                 sbt->sbt(),
        //                 directions.size(), 1, 1) );
        //cudaDeviceSynchronize();
        //CUDA_CHECK_LAST();

        oculusReceiver->reduce_samples();

        optixRenderer->points().set_device_data(optixPoints.size(),
            reinterpret_cast<const typename plt::GLMesh::Point*>(optixPoints.data()));
        optixRenderer->set_pose(pose);
        trace->add_pose(pose);
        trace->add_pose(pose * oculusToGL);

        pingRenderer->set_data(meta, pingData);
        simRenderer->set_data(oculusReceiver->target());

        glSimView->set_pose(pose * oculusToGL);

        GL_CHECK_LAST();
        frameBuffer->bind(GL_FRAMEBUFFER);
        GL_CHECK_LAST();
        glClearColor(0.0,0.0,0.0,0.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        renderer2->draw(glSimView);
        //glBindRenderbuffer(GL_RENDERBUFFER, 0);
        //glBindTexture(GL_TEXTURE_2D, 0);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        GL_CHECK_LAST();

        frameBuffer->bind(GL_READ_FRAMEBUFFER);
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        pixelOutput.bind(GL_PIXEL_PACK_BUFFER);
        glReadPixels(0, 0,
                     glSim.window_shape().width,
                     glSim.window_shape().height,
                     GL_RGBA, GL_FLOAT, nullptr);
        GL_CHECK_LAST();
        glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);

        //glReceiver->samples().resize(pixelOutput
        glReceiver->reconfigure(meta, pingData);
        pixelOutput.to_device_vector(glReceiver->samples());
        glReceiver->reduce_samples();

        simRenderer2->set_data(glReceiver->target());

        //fbRenderer->texture()->set_image(glSim.window_shape(), pixelOutput);

        //glSim.draw();// crashing when here
        display.draw();
        //glSim.draw();// crashing when here
        sonarDisplay.draw();
        //glSim.draw();// crashing when here
        simDisplay.draw();
        simDisplay2.draw();
        glSim.draw(); //but not here

        //std::this_thread::sleep_for(50ms);
    }

    return 0;
}

#include <random>
DeviceVector<float> dtm_phases(const MeshGeometry::Ptr& mesh)
{
    std::vector<float> phases(mesh->primitive_count());
    
    std::mt19937 randGenerator((std::random_device())());
    std::uniform_real_distribution<float> distribution(-M_PI, M_PI);
    for(auto& v : phases) {
        v = distribution(randGenerator);
    }

    return DeviceVector<float>(phases);
}
