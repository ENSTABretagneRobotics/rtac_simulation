#include <iostream>
#include <thread>
using namespace std;

#include <rtac_simulation/helpers/OculusRosbagIterator.h>
#include <rtac_simulation/helpers/OculusRenderer.h>

#include <narval_oculus/Oculus.h>

#include <rtac_base/files.h>
#include <rtac_base/external/obj_codec.h>
using namespace rtac::files;
#include <rtac_base/navigation.h>
using namespace rtac::navigation;
#include <rtac_base/types/Mesh.h>
#include <rtac_base/types/Pose.h>
using Mesh = rtac::Mesh<>;
using Pose = rtac::Pose<float>;

#include <rtac_base/cuda/vector_utils.h>

#include <rtac_base/types/Pose.h>
using Pose = rtac::Pose<float>;
using Vec3       = Pose::Vec3;
using Mat3       = Pose::Mat3;
using Mat4       = Pose::Mat4;
using Quaternion = Pose::Quat;
#include <rtac_base/interpolation.h>
using namespace rtac::algorithm;

#include <rtac_base/containers/HostVector.h>

#include <rtac_base/cuda/Texture2D.h>
#include <rtac_base/cuda/DeviceVector.h>
#include <rtac_base/cuda/DeviceObject.h>
using namespace rtac::cuda;

#include <rtac_display/samples/Display3D.h>
#include <rtac_display/renderers/MeshRenderer.h>
#include <rtac_display/renderers/PointCloudRenderer.h>
#include <rtac_display/renderers/ImageRenderer.h>
#include <rtac_display/renderers/FrameInstances.h>
#include <rtac_display/renderers/FanRenderer.h>
#include <rtac_display/renderers/Frame.h>
namespace plt = rtac::display;

#include <rtac_simulation/helpers/PolarTargetRenderer.h>
#include <rtac_simulation/RayCaster.h>
#include <rtac_simulation/Binner.h>
#include <rtac_simulation/factories/utilities.h>
#include <rtac_simulation/factories/SensorInfoFactory.h>
#include <rtac_simulation/SensorInstance.h>

#include <rtac_optix/utils.h>
#include <rtac_optix/Context.h>
#include <rtac_optix/Pipeline.h>
#include <rtac_optix/MeshGeometry.h>
#include <rtac_optix/ObjectInstance.h>
#include <rtac_optix/GroupInstance.h>
#include <rtac_optix/ShaderBindingTable.h>
#include <rtac_optix/ShaderBinding.h>
using namespace rtac::optix;

#include <rtac_simulation/examples/blueprint_oculus.h>

#include <ray_caster02_rtac_simulation/ptx_files.h>

#include "oculus_sim.h"
//using namespace rtac::simulation;

MeshGeometry::Ptr geometry_from_mesh(const Context::Ptr& ctx, const Mesh& mesh);
DeviceVector<float> dtm_phases(const MeshGeometry::Ptr& mesh);

int main()
{
    auto dtmPath = find_one(".*models3d/pyramide2_test01/.*.obj");
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

    auto ptxFiles = ray_caster02_rtac_simulation::get_ptx_files();
    optix_init();

    //auto raycaster = rtac::simulation::PolarRayCaster::Create();
    auto raycaster = rtac::simulation::RayCaster::Create();
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

    auto emitter = rtac::simulation::Emitter2::Create(0.1f, 140.0f, 100.0f, 
    //auto emitter = rtac::simulation::Emitter2::Create(10.0f, 140.0f, 100.0f, 
        rtac::simulation::Directivity::from_sinc_parameters(130.0f, 20.0f));
    auto receiver = rtac::simulation::Receiver2<rtac::simulation::SimSample2D>::Create(emitter->directivity());
    auto oculusSensor = rtac::simulation::OculusSensor::Create();
    auto finder = rtac::simulation::FileFinder::Get({std::string(RTAC_TEST_CONFIG)});
    auto filename = finder->find_one(".*oculus_M1200d_1.yaml");
    std::cout << "config file : " << filename << std::endl;
    //auto sensorInfo = rtac::simulation::SensorInfoFactory2D::Make(YAML::LoadFile(filename));
    auto sensorInfo = rtac::simulation::SensorInfoFactory2D::Make2(YAML::LoadFile(filename));
    auto oculusSensor2 = rtac::simulation::SensorModel2D_Complex::Create(sensorInfo);
    auto oculusSensor3 = rtac::simulation::SensorInstance::Create(sensorInfo);
    //rtac::simulation::Binner binner;

    DeviceVector<float3> optixPoints;

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
    auto renderer = display.create_renderer<plt::MeshRenderer>(display.view());
    renderer->mesh() = glMesh;
    auto optixRenderer = display.create_renderer<plt::PointCloudRenderer>(display.view());
    optixRenderer->set_color({0.5,0.0,0.0,1.0});
    auto trace = display.create_renderer<plt::FrameInstances>(display.view());

    plt::Display sonarDisplay(display.context());
    sonarDisplay.disable_frame_counter();
    auto pingRenderer = sonarDisplay.create_renderer<plt::OculusRenderer>(plt::View::Create());

    plt::Display simDisplay(display.context());
    simDisplay.disable_frame_counter();
    //auto simRenderer = simDisplay.create_renderer<plt::PolarTargetRenderer>(plt::View::Create());
    auto simRenderer = simDisplay.create_renderer<plt::FanRenderer>(plt::View::Create());
    //simRenderer->set_direction(plt::FanRenderer::Direction::Up);
    //auto simRenderer = simDisplay.create_renderer<plt::ImageRenderer>(plt::View::Create());
    //simRenderer->enable_colormap();

    //auto oculusDatum = bag.next();
    //for(int i = 0; i < 100; i++) {
    //    oculusDatum = bag.next();
    //}
    int screenshotCount = 0;
    int loopCount = 0;
    while(!display.should_close() &&
          !simDisplay.should_close() &&
          !sonarDisplay.should_close())
    {
        auto oculusDatum = bag.next();

        auto meta     = oculusDatum.ping_metadata();
        auto pingData = oculusDatum.ping_data();
        auto pose     = oculusDatum.pose();

        //oculusEmitter->pose()  = pose;
        //oculusReceiver->pose() = pose;
        //oculusReceiver->samples().resize(directions.size());

        //oculusReceiver->reconfigure(meta, pingData);
        oculusSensor->reconfigure(meta, pingData);
        oculusSensor2->set_ranges(meta.fireMessage.range, meta.nRanges);
        oculusSensor3->set_ranges(meta.fireMessage.range, meta.nRanges);

        emitter->pose()  = pose;
        receiver->pose() = pose;
        receiver->samples().resize(emitter->size());
        //receiver->reconfigure(meta, pingData);

        //raycaster->trace(oculusEmitter, oculusReceiver,
        //                 directions, optixPoints);
        raycaster->trace(emitter, receiver, optixPoints);
        receiver->sort_received();
        oculusSensor->sensor()->reduce_samples(receiver->samples());
        oculusSensor2->reduce_samples(receiver->samples());
        rtac::Image<rtac::Complex<float>,  rtac::cuda::DeviceVector> out;
        oculusSensor3->reduce_samples(receiver->samples(), out);

        simRenderer->set_range(oculusSensor->sensor()->data().height_dim().bounds());
        simRenderer->set_bearings(oculusSensor->sensor()->data().width_dim().size(),
                                  oculusSensor->sensor()->data().width_dim().data());
        //auto tmp1 = abs(oculusSensor->sensor()->data().container());
        //auto tmp1 = abs(oculusSensor2->data());
        auto tmp1 = abs(out.container());
        simRenderer->set_data({oculusSensor->sensor()->data().width(),
                               oculusSensor->sensor()->data().height()},
                              plt::GLVector<float>(rescale(tmp1, 0.0f, 10.0f)), false);
                              //plt::GLVector<float>(rescale(tmp1, 0.0f, 1.0f)), false);

        //simRenderer->texture()->set_image({oculusSensor->sensor()->width(),
        //                                   oculusSensor->sensor()->height()},
        //                                  plt::GLVector<float>(rescale(tmp1, 0.0f, 1.0f)));
        //simRenderer->texture()->set_image({oculusSensor->sensor()->point_spread_function().width(),
        //                                   oculusSensor->sensor()->point_spread_function().height()},
        //                                  plt::GLVector<float>(rescale(render_kernel(oculusSensor->sensor()->point_spread_function()).container(), 0.0f, 1.0f)));

        //rtac::cuda::DeviceVector<rtac::VectorView<const rtac::simulation::SimSample2D>> bins;
        //binner.reconfigure(meta.nRanges, {0.0f, (float)meta.fireMessage.range});
        //binner.compute(bins, receiver->samples());

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

        //oculusReceiver->reduce_samples();

        optixRenderer->points().copy_from_cuda(optixPoints.size(),
            reinterpret_cast<const typename plt::GLMesh::Point*>(optixPoints.data()));
        optixRenderer->set_pose(pose);
        trace->add_pose(pose);

        pingRenderer->set_data(meta, pingData);
        //simRenderer->set_data(oculusReceiver->target());

        display.draw();
        sonarDisplay.draw();
        simDisplay.draw();

        //rtac::Image<rtac::Point3<unsigned char>, std::vector> screenshot;
        //sonarDisplay.take_screenshot(screenshot);
        //{
        //    std::ostringstream oss;
        //    oss << "output/" << setfill('0') << setw(5)
        //        << screenshotCount << "_sonar.ppm";
        //    for(int i = 0; i < screenshot.size(); i++) {
        //        screenshot[i].x = 255 - screenshot[i].x;
        //        screenshot[i].y = 255 - screenshot[i].y;
        //        screenshot[i].z = 255 - screenshot[i].z;
        //    }
        //    rtac::files::write_ppm(oss.str(),
        //                           screenshot.shape().width, 
        //                           screenshot.shape().height,
        //                           (const unsigned char*)screenshot.data());
        //                     
        //}
        //simDisplay.take_screenshot(screenshot);
        //{
        //    std::ostringstream oss;
        //    oss << "output/" << setfill('0') << setw(5)
        //        << screenshotCount << "_sim.ppm";
        //    for(int i = 0; i < screenshot.size(); i++) {
        //        screenshot[i].x = 255 - screenshot[i].x;
        //        screenshot[i].y = 255 - screenshot[i].y;
        //        screenshot[i].z = 255 - screenshot[i].z;
        //    }
        //    rtac::files::write_ppm(oss.str(),
        //                           screenshot.shape().width, 
        //                           screenshot.shape().height, 
        //                           (const unsigned char*)screenshot.data());
        //}
        screenshotCount++;

        std::this_thread::sleep_for(50ms);
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
