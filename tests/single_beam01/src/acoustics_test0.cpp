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
#include <rtac_display/renderers/SimplePlot.h>
namespace plt = rtac::display;

#include <rtac_simulation/RayCaster.h>
#include <rtac_simulation/SensorInstance.h>
#include <rtac_simulation/SensorInstance1D.h>
#include <rtac_simulation/OptixSimulation.h>
#include <rtac_simulation/factories/EmitterFactory.h>
#include <rtac_simulation/factories/SensorInfoFactory.h>
using namespace rtac::simulation;

int main()
{
    auto dtmPath = find_one(".*models3d/pyramide2_test01/.*.obj");
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

    //auto simulation = OptixSimulation1::Create("oculus_M1200d_1_emitter.yaml",
    //                                           "oculus_M1200d_1_receiver.yaml");
    //auto oculusSensor3 = std::dynamic_pointer_cast<SensorInstance2D_Complex>(simulation->receiver().ptr());

    auto simulation = OptixSimulation1::Create("EA400_200KHz_emitter.yaml",
                                               "EA400_200KHz_receiver.yaml");
    auto ea400 = std::dynamic_pointer_cast<SensorInstance1D_Complex>(simulation->receiver().ptr());

    //auto filename = FileFinder::Get()->find_one("EA400_200KHz_receiver.yaml");
    //std::cout << "config file ea400 : " << filename << std::endl;
    //auto ea400 = SensorFactory::Make(YAML::LoadFile(filename));

    rtac::external::ObjLoader parser(dtmPath);
    parser.load_geometry();
    auto mesh = parser.create_single_mesh<HostMesh<>>();
    cout << "Number of points : " << mesh->points().size() << endl;
    cout << "Number of faces  : " << mesh->faces().size() << endl;

    simulation->add_object(DeviceMesh<>::Create(*mesh));

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
    //auto simRenderer = simDisplay.create_renderer<plt::FanRenderer>(plt::View::Create());
    auto simRenderer = simDisplay.create_renderer<plt::SimplePlot>(plt::View::Create());

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

        simulation->emitter().pose()  = pose;
        simulation->receiver().pose() = pose;
        simulation->receiver().set_ranges(meta.fireMessage.range, meta.nRanges);
        simulation->run();

        //simRenderer->set_range(oculusSensor3->ranges().bounds());
        //simRenderer->set_bearings(oculusSensor3->width(), oculusSensor3->bearings().data());
        //auto tmp1 = abs(oculusSensor3->output().container());
        //simRenderer->set_data({oculusSensor3->width(),
        //                       oculusSensor3->height()},
        //                      //plt::GLVector<float>(rescale(tmp1, 0.0f, 10.0f)), false);
        //                      plt::GLVector<float>(rescale(tmp1, 0.0f, 1.2f)), false);
        //                      //plt::GLVector<float>(rescale(tmp1, 0.0f, 1.0f)), false);

        auto tmp1 = real(ea400->output());
        simRenderer->set_data(tmp1);


        optixRenderer->points().copy_from_cuda(optixPoints.size(),
            reinterpret_cast<const typename plt::GLMesh::Point*>(optixPoints.data()));
        optixRenderer->set_pose(pose);
        trace->add_pose(pose);

        pingRenderer->set_data(meta, pingData);

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

//#include <random>
//DeviceVector<float> dtm_phases(const MeshGeometry::Ptr& mesh)
//{
//    std::vector<float> phases(mesh->primitive_count());
//    
//    std::mt19937 randGenerator((std::random_device())());
//    std::uniform_real_distribution<float> distribution(-M_PI, M_PI);
//    for(auto& v : phases) {
//        v = distribution(randGenerator);
//    }
//
//    return DeviceVector<float>(phases);
//}
