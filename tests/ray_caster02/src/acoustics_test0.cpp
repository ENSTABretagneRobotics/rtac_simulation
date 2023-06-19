#include <iostream>
#include <thread>
#include <fstream>
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
#include <rtac_base/serialization/sonar_ping.h>

#include <rtac_base/cuda/Texture2D.h>
#include <rtac_base/cuda/CudaVector.h>
#include <rtac_base/cuda/CudaPing.h>
using namespace rtac::cuda;

#include <rtac_display/samples/Display3D.h>
#include <rtac_display/renderers/MeshRenderer.h>
#include <rtac_display/renderers/PointCloudRenderer.h>
#include <rtac_display/renderers/ImageRenderer.h>
#include <rtac_display/renderers/FrameInstances.h>
#include <rtac_display/renderers/FanRenderer.h>
#include <rtac_display/renderers/Frame.h>
namespace plt = rtac::display;

#include <rtac_simulation/RayCaster.h>
#include <rtac_simulation/SensorInstance.h>
#include <rtac_simulation/OptixSimulation.h>
//#include <rtac_simulation/sinks/Sink2D.h>
//#include <rtac_simulation/sinks/DisplaySink2D.h>
#include <rtac_simulation/sinks/FileSink.h>
#include <rtac_simulation/sinks/DisplaySink.h>
#include <rtac_simulation/PoseSource.h>
#include <rtac_simulation/factories/EmitterFactory.h>
#include <rtac_simulation/factories/SensorInfoFactory.h>
using namespace rtac::simulation;


int main()
{
    auto dtmPath = find_one(".*models3d/pyramide2_test01_2/.*.obj");
    //auto dtmPath = find_one(".*models3d/pyramide2_test01_2_subsampled/.*.obj");
    auto bagPath = find_one(".*pyramide2_matisse_positions.bag");
    //auto dtmPath = find_one(".*models3d/pyramide2_test01", "/media/pnarvor/Samsung_T5/submeeting/save_sabre01");
    //auto bagPath = find_one(".*pyramide2_matisse_positions.bag", "/media/pnarvor/Samsung_T5/submeeting/save_sabre01");

    cout << "DTM path : " << dtmPath << endl;

    auto Roculus = rtac::Pose<float>::from_rotation_matrix(
        Eigen::AngleAxisf(0.5f*M_PI, Eigen::Vector3f::UnitZ()).toRotationMatrix());
    OculusRosbagIterator bag(bagPath, 
                                               "/oculus_sonar/ping",
                                               "/matisse_pose",
                                               Roculus);

    rtac::external::ObjLoader parser(dtmPath);
    parser.load_geometry();

    auto mesh = parser.create_single_mesh<HostMesh<>>();

    auto simulation = OptixSimulation1::Create("oculus_M1200d_1_emitter.yaml",
                                               "oculus_M1200d_1_receiver.yaml");
    auto oculusSensor3 = std::dynamic_pointer_cast<SensorInstance2D_2<rtac::Complex<float>>>(simulation->receiver().ptr());
    //auto oculusSensor3 = std::dynamic_pointer_cast<SensorInstance2D_Complex>(simulation->receiver().ptr());
    simulation->add_object(DeviceMesh<>::Create(*mesh));

    cout << "Number of points : " << mesh->points().size() << endl;
    cout << "Number of faces  : " << mesh->faces().size() << endl;
    cout << "Number of rays   : " << simulation->emitter().ray_count() << std::endl;

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

    plt::Display sonarDisplay(800, 600, "Real Sonar", display.context());
    sonarDisplay.disable_frame_counter();
    auto pingRenderer = sonarDisplay.create_renderer<plt::OculusRenderer>(plt::View::Create());

    //CudaPing2D<rtac::Complex<float>> ping(rtac::Linspace<float>(0.0,1.0,0),
    //                                      rtac::HostVector<float>::linspace(-1.0,1.0,2));

    simulation->add_sink(rtac::simulation::FileSink::Create("output.rtac", true));
    auto displaySink = simulation->add_sink(rtac::simulation::DisplaySink::Create(display.context(), "Simulated Sonar"));

    auto poseSource = rtac::simulation::PoseSourceStatic::Create();
    simulation->set_emitter_pose_source(poseSource);
    simulation->set_receiver_pose_source(poseSource);

    uint64_t t0 = 0;
    std::ostringstream ossPoses;
    ossPoses << "# quat,tx,ty,tz,qw,qx,qy,qz\r\n";

    int screenshotCount = 0;
    int loopCount = 0;
    while(!display.should_close() &&
          !displaySink->window().should_close() &&
          !sonarDisplay.should_close())
    {
        auto oculusDatum = bag.next();
        auto meta     = oculusDatum.ping_metadata();
        auto pingData = oculusDatum.ping_data();
        auto pose     = oculusDatum.pose();

        auto stamp = oculusDatum.poseMsg_->header.stamp;
        uint64_t t = 1000*stamp.sec + 1.0e-6*stamp.nsec;
        if(t == t0) {
            std::cout << "loop" << std::endl;
            std::ofstream f("poses.csv");
            f << ossPoses.str();
        }
        if(t0 == 0) t0 = t;
        ossPoses << pose.encode_string("quat") << "\r\n";

        //simulation->emitter().pose()  = pose;
        //simulation->receiver().pose() = pose;
        poseSource->set_pose(pose);
        //simulation->receiver().set_ranges(meta.fireMessage.range, meta.nRanges);
        simulation->receiver().set_ranges(1.05*meta.fireMessage.range, meta.nRanges);
        simulation->run();

        optixRenderer->points().copy_from_cuda(simulation->hit_points().size(),
            reinterpret_cast<const typename plt::GLMesh::Point*>(simulation->hit_points().data()));
        trace->add_pose(pose);

        pingRenderer->set_data(meta, pingData);

        display.draw();
        sonarDisplay.draw();

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

//#include <random>
//CudaVector<float> dtm_phases(const MeshGeometry::Ptr& mesh)
//{
//    std::vector<float> phases(mesh->primitive_count());
//    
//    std::mt19937 randGenerator((std::random_device())());
//    std::uniform_real_distribution<float> distribution(-M_PI, M_PI);
//    for(auto& v : phases) {
//        v = distribution(randGenerator);
//    }
//
//    return CudaVector<float>(phases);
//}
