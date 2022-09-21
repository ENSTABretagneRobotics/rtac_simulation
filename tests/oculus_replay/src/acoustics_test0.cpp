#include <iostream>
#include <thread>
using namespace std;

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <narval_oculus/Oculus.h>
#include <narval_oculus/helpers/OculusRenderer.h>
#include <oculus_sonar/OculusPing.h>

#include <rtac_base/files.h>
#include <rtac_base/external/obj_codec.h>
using namespace rtac::files;
#include <rtac_base/navigation.h>
using namespace rtac::navigation;
#include <rtac_base/types/Mesh.h>
#include <rtac_base/types/Pose.h>
using Mesh = rtac::types::Mesh<>;
using Pose = rtac::types::Pose<float>;

#include <rtac_base/types/Pose.h>
using Pose = rtac::types::Pose<float>;
using Vec3       = Pose::Vec3;
using Mat3       = Pose::Mat3;
using Mat4       = Pose::Mat4;
using Quaternion = Pose::Quaternion;
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

#include <oculus_replay_rtac_simulation/ptx_files.h>

#include "oculus_sim.h"
using namespace narval;

template <typename T>
using BoostPtr    = boost::shared_ptr<T>;
using OculusMsg   = BoostPtr<oculus_sonar::OculusPing>;
//using PoseMsg     = BoostPtr<nav_msgs::Odometry>;
using PoseMsg     = BoostPtr<geometry_msgs::PoseStamped>;
using OculusDatum = std::pair<OculusMsg, PoseMsg>;

MeshGeometry::Ptr geometry_from_mesh(const Context::Ptr& ctx, const Mesh& mesh);
DeviceVector<float> dtm_phases(const MeshGeometry::Ptr& mesh);
Pose sonar_pose();

int main()
{
    auto dtmPath = find_one(".*models3d/pyramide2_test01");
    auto bagPath = find_one(".*pyramide2_matisse_positions.bag");
    //auto dtmPath = find_one(".*models3d/pyramide2_test01", "/media/pnarvor/Samsung_T5/submeeting/save_sabre01");
    //auto bagPath = find_one(".*pyramide2_matisse_positions.bag", "/media/pnarvor/Samsung_T5/submeeting/save_sabre01");

    cout << "DTM path : " << dtmPath << endl;

    rosbag::Bag bag;
    cout << "Opening rosbag " << bagPath << " ... " << flush;
    bag.open(bagPath, rosbag::bagmode::Read);
    cout << "Done." << endl << flush;
    std::vector<std::string> topics({"/matisse_pose", "/oculus_sonar/ping"});
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    rtac::external::ObjLoader parser(dtmPath);
    parser.load_geometry();

    //auto mesh = HostMesh<>::from_ply(dtmPath);
    auto mesh = parser.create_single_mesh<HostMesh<>>();
    cout << "Number of points : " << mesh->points().size() << endl;
    cout << "Number of faces  : " << mesh->faces().size() << endl;

    auto ptxFiles = oculus_replay_rtac_simulation::get_ptx_files();
    optix_init();
    auto context  = Context::Create();
    auto pipeline = Pipeline::Create(context);
    pipeline->compile_options().numPayloadValues = 8;
    pipeline->compile_options().exceptionFlags = OPTIX_EXCEPTION_FLAG_DEBUG;
    auto module   = pipeline->add_module("oculus_sonar", ptxFiles["src/oculus_sim.cu"]);
    auto raygen   = pipeline->add_raygen_program("__raygen__oculus_sonar", "oculus_sonar");
    auto miss     = pipeline->add_miss_program("__miss__oculus_sonar", "oculus_sonar");
    auto hitSonar = pipeline->add_hit_programs();
    //hitSonar->set_closesthit({"__closesthit__oculus_sonar", pipeline->module("oculus_sonar")});
    
    DeviceVector<float> dtmPhases;
    ObjectInstance::Ptr dtmObject; {
        //auto geom = geometry_from_mesh(context, mesh);
        auto geom = MeshGeometry::Create(context, DeviceMesh<>::Create(*mesh));
        geom->material_hit_setup({OPTIX_GEOMETRY_FLAG_NONE});
        geom->enable_vertex_access();
        dtmPhases = dtm_phases(geom);

        dtmObject = ObjectInstance::Create(geom);

        hitSonar->set_closesthit({"__closesthit__oculus_sonar_phased",
                                  pipeline->module("oculus_sonar")});
        dtmObject->add_material(SonarMaterial::Create(
                                hitSonar, PhaseData({dtmPhases.size(), dtmPhases.data()})));
    }

    auto topObject = GroupInstance::Create(context);
    topObject->add_instance(dtmObject);

    auto sbt = ShaderBindingTable::Create(Raytypes::RaytypeCount);
    sbt->set_raygen_record(ShaderBinding<void>::Create(raygen));
    sbt->add_miss_record(SonarMissMaterial::Create(miss));
    sbt->add_object(dtmObject);

    auto directions = rtac::simulation::Emitter<float>::generate_polar_directions(0.1f,
                                                                                  140.0f,
                                                                                  100.0f);
    auto Roculus = rtac::types::Pose<float>::from_rotation_matrix(
        Eigen::AngleAxisf(0.5f*M_PI, Eigen::Vector3f::UnitZ()).toRotationMatrix());
    
    auto oculusEmitter = rtac::simulation::Emitter<float>::Create(
        directions.container(), 
        rtac::simulation::Directivity::from_sinc_parameters(130.0f, 20.0f));
    auto oculusReceiver = rtac::simulation::OculusReceiver::Create();

    DeviceObject<Params> optixParams;

    optixParams.topObject = *topObject;

    DeviceVector<float3>  optixPoints;

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

    auto sonarPose = sonar_pose();

    plt::Display sonarDisplay(display.context());
    sonarDisplay.disable_frame_counter();
    auto pingRenderer = sonarDisplay.create_renderer<plt::OculusRenderer>(plt::View::New());

    plt::Display simDisplay(display.context());
    simDisplay.disable_frame_counter();
    auto simRenderer = simDisplay.create_renderer<plt::PolarTargetRenderer>(plt::View::New());

    OculusDatum oculusDatum;

    rtac::types::Image<rtac::types::Point3<unsigned char>, std::vector> screenshot;

    int count = 0;
    auto it = view.begin();
    int screenshotCount = 0;
    while(!display.should_close() &&
          !simDisplay.should_close() &&
          !sonarDisplay.should_close()) {
        if(auto msg = it->instantiate<geometry_msgs::PoseStamped>()) {
            oculusDatum.second = msg;
        }
        if(auto msg = it->instantiate<oculus_sonar::OculusPing>()) {
            oculusDatum.first = msg;
        }

        it++;
        if(it == view.end()) {
            count = 0;
            it = view.begin();
            oculusDatum.first  = nullptr;
            oculusDatum.second = nullptr;
            screenshotCount = 0;
        }
        if(!oculusDatum.first || !oculusDatum.second) {
            continue;
        }

        auto pingMsg = oculusDatum.first;
        auto meta = (const OculusSimplePingResult*)pingMsg->data.data();
        oculusReceiver->reconfigure(*meta, pingMsg->data.data());

        auto rosPose = oculusDatum.second->pose;

        plt::Frame::Pose pose({(float)rosPose.position.x,
                               (float)rosPose.position.y,
                               (float)rosPose.position.z},
                              {(float)rosPose.orientation.w,
                               (float)rosPose.orientation.x,
                               (float)rosPose.orientation.y,
                               (float)rosPose.orientation.z});

        optixPoints.resize(directions.size());

        optixParams.outputPoints  = optixPoints.data();

        oculusEmitter->pose() = pose * Roculus;

        oculusReceiver->pose() = pose * Roculus;
        oculusReceiver->samples().resize(directions.size());

        optixParams.emitter    = oculusEmitter->view();
        optixParams.receiver   = oculusReceiver->view();
        optixParams.directions = directions.data();

        optixParams.update_device();
        CUDA_CHECK_LAST();

        OPTIX_CHECK( optixLaunch(*pipeline, 0,
                         (CUdeviceptr)optixParams.device_ptr(), sizeof(Params),
                         sbt->sbt(),
                         directions.size(), 1, 1) );
        cudaDeviceSynchronize();
        CUDA_CHECK_LAST();

        oculusReceiver->reduce_samples();

        optixRenderer->points().set_device_data(optixPoints.size(),
            reinterpret_cast<const typename plt::GLMesh::Point*>(optixPoints.data()));
        optixRenderer->set_pose(pose * Roculus);
        trace->add_pose(pose * Roculus);

        pingRenderer->set_data(*meta, (const uint8_t*)pingMsg->data.data());
        simRenderer->set_data(oculusReceiver->target());

        display.draw();
        sonarDisplay.draw();
        simDisplay.draw();

        screenshotCount++;
        oculusDatum.first  = nullptr;
        oculusDatum.second = nullptr;

        //std::this_thread::sleep_for(50ms);
    }

    return 0;
}

#include <random>
DeviceVector<float> dtm_phases(const MeshGeometry::Ptr& mesh)
{
    //std::vector<float> phases(mesh->num_points());
    //size_t N = 0;//
    //if(mesh->num_//faces() > 0) {
    //    N = mesh-//>num_faces();
    //} else {
    //    N = mesh->num_points() / 3;
    //}
    std::vector<float> phases(mesh->primitive_count());
    
    std::mt19937 randGenerator((std::random_device())());
    std::uniform_real_distribution<float> distribution(-M_PI, M_PI);
    for(auto& v : phases) {
        v = distribution(randGenerator);
    }

    return DeviceVector<float>(phases);
}

Pose sonar_pose()
{
    Pose pose({0.028f, 0.275f, 0.272f},
        Pose::Quaternion(Eigen::AngleAxisf(0.5f*M_PI, Eigen::Vector3f::UnitZ())) *
        Pose::Quaternion(Eigen::AngleAxisf(-3.0f*M_PI / 180.0, Eigen::Vector3f::UnitY())) *
        Pose::Quaternion(0.9915618937147881f, 0.0f, -0.12963414261969486f, 0.0f));

    return pose;
}
