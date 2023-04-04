#include <iostream>
#include <thread>
using namespace std;

#include <rtac_simulation/helpers/OculusRosbagIterator.h>
#include <rtac_simulation/helpers/OculusRenderer.h>

#include <narval_oculus/Oculus.h>

#include <rtac_base/external/metashape.h>
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

#include <rtac_simulation/RayCaster.h>
#include <rtac_simulation/SensorInstance.h>
#include <rtac_simulation/OptixSimulation.h>
#include <rtac_simulation/factories/EmitterFactory.h>
#include <rtac_simulation/factories/SensorInfoFactory.h>
using namespace rtac::simulation;

int main()
{
    auto obj = rtac::files::find_one(".*models3d_decimated/test_oculus_4pyr.obj");
    auto xml = rtac::files::find_one(".*models3d_decimated/test_oculus_4pyr.xml");

    //auto obj = rtac::files::find_one(".*models3d_decimated/test_oculus_cercle_pilier_1.obj");
    //auto xml = rtac::files::find_one(".*models3d_decimated/test_oculus_cercle_pilier_1.xml");

    //auto obj = rtac::files::find_one(".*models3d_decimated/test_oculus_cercle_pilier_2.obj");
    //auto xml = rtac::files::find_one(".*models3d_decimated/test_oculus_cercle_pilier_2.xml");

    // auto obj = rtac::files::find_one(".*models3d_decimated/test_oculus_fosse.obj");
    // auto xml = rtac::files::find_one(".*models3d_decimated/test_oculus_fosse.xml");

    //auto obj = rtac::files::find_one(".*models3d_decimated/test_oculus_mur_pyr.obj");
    //auto xml = rtac::files::find_one(".*models3d_decimated/test_oculus_mur_pyr.xml");

    // auto obj = rtac::files::find_one(".*models3d_decimated/test_oculus_pyr_pil_metro_1338.obj");
    // auto xml = rtac::files::find_one(".*models3d_decimated/test_oculus_pyr_pil_metro_1338.xml");


    cout << "3D model  : " << obj << endl;
    cout << "Pose file : " << xml << endl;

    auto Roculus = rtac::Pose<float>::from_rotation_matrix(
        Eigen::AngleAxisf(0.5f*M_PI, Eigen::Vector3f::UnitX()).toRotationMatrix() *
        Eigen::AngleAxisf(0.5f*M_PI, Eigen::Vector3f::UnitZ()).toRotationMatrix());

    rtac::external::MetashapeOutput metashape;
    metashape.load_file(xml);

    rtac::external::ObjLoader parser(obj);
    parser.load_geometry();

    auto mesh = parser.create_single_mesh<HostMesh<>>();

    auto simulation = OptixSimulation1::Create("oculus_M1200d_1_emitter.yaml",
                                               "oculus_M1200d_1_receiver.yaml");
    auto oculusSensor3 = std::dynamic_pointer_cast<SensorInstance2D_Complex>(simulation->receiver().ptr());
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

    plt::Display simDisplay(display.context());
    simDisplay.disable_frame_counter();
    auto simRenderer = simDisplay.create_renderer<plt::FanRenderer>(plt::View::Create());

    plt::Display simDisplay2(display.context());
    simDisplay2.disable_frame_counter();
    auto simRenderer2 = simDisplay2.create_renderer<plt::ImageRenderer>(plt::View::Create());
    simRenderer2->enable_colormap();

    auto poseIt = metashape.poses(0).begin();

    int screenshotCount = 0;
    int loopCount = 0;
    while(!display.should_close() &&
          !simDisplay.should_close())
    {
        auto pose = (*poseIt) * Roculus;
        if(++poseIt == metashape.poses(0).end()) {
            poseIt = metashape.poses(0).begin();
        }

        simulation->emitter().pose()  = pose;
        simulation->receiver().pose() = pose;
        //simulation->receiver().set_ranges(meta.fireMessage.range, meta.nRanges);
        simulation->run();

        simRenderer->set_range(oculusSensor3->ranges().bounds());
        simRenderer->set_bearings(oculusSensor3->width(), oculusSensor3->bearings().data());
        auto tmp1 = abs(oculusSensor3->output().container());
        tmp1 = log(tmp1 += 1.0e-4f*max(tmp1));
        simRenderer->set_data({oculusSensor3->width(),
                               oculusSensor3->height()},
                              //plt::GLVector<float>(rescale(tmp1, 0.0f, 10.0f)), false);
                              //plt::GLVector<float>(rescale(tmp1, 0.0f, 1.2f)), false);
                              //plt::GLVector<float>(rescale(tmp1, 0.0f, 1.0f)), false);
                              plt::GLVector<float>(rescale(tmp1, 0.0f, 0.9f)), false);

        //simRenderer2->texture()->set_image({oculusSensor3->width(), oculusSensor3->height()},
        //                      plt::GLVector<float>(rescale(tmp1, 0.0f, 10.0f)));
        //                      //plt::GLVector<float>(rescale(tmp1, 0.0f, 1.2f)));
        //                      //plt::GLVector<float>(rescale(tmp1, 0.0f, 1.0f)));

        optixRenderer->points().copy_from_cuda(simulation->hit_points().size(),
            reinterpret_cast<const typename plt::GLMesh::Point*>(simulation->hit_points().data()));
        //optixRenderer->set_pose(pose);
        trace->add_pose(pose);

        display.draw();
        simDisplay.draw();
        //simDisplay2.draw();

        screenshotCount++;

        //std::this_thread::sleep_for(50ms);
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
