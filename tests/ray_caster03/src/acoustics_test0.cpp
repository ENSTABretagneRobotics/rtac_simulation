#include <iostream>
#include <thread>
#include <fstream>
using namespace std;

#include <rtac_base/files.h>
#include <rtac_base/external/obj_codec.h>
#include <rtac_base/types/Mesh.h>
#include <rtac_base/types/Pose.h>
using Mesh = rtac::Mesh<>;
using Pose = rtac::Pose<float>;
using namespace rtac;

#include <rtac_base/cuda/DeviceMesh.h>
using namespace rtac::cuda;

#include <rtac_simulation/OptixSimulation.h>
#include <rtac_simulation/SensorInstance2D.h>
#include <rtac_simulation/sinks/FileSink.h>
#include <rtac_simulation/sinks/DisplaySink.h>
#include <rtac_simulation/PoseSource.h>
#include <rtac_simulation/factories/utilities.h>
#include <rtac_simulation/factories/PoseSourceFactory.h>
using namespace rtac::simulation;

int main()
{
    auto simulation = OptixSimulation1::Create("oculus_M1200d_1_emitter.yaml",
                                               "oculus_M1200d_1_receiver.yaml");
    auto oculusSensor3 = std::dynamic_pointer_cast<SensorInstance2D_2<Complex<float>>>(simulation->receiver().ptr());

    auto poseSource = PoseSourceFactory::Make("pose_source.yaml");
    simulation->set_emitter_pose_source(poseSource);
    simulation->set_receiver_pose_source(poseSource);

    auto dtmPath = FileFinder::Get()->find_one("st_raphael_01.obj");
    cout << "DTM path : " << dtmPath << endl;
    external::ObjLoader parser(dtmPath);
    parser.load_geometry();
    auto mesh = parser.create_single_mesh<HostMesh<>>();
    simulation->add_object(DeviceMesh<>::Create(*mesh));

    cout << "Number of points : " << mesh->points().size() << endl;
    cout << "Number of faces  : " << mesh->faces().size() << endl;
    cout << "Number of rays   : " << simulation->emitter().ray_count() << std::endl;

    auto displaySink = simulation->add_sink(rtac::simulation::DisplaySink::Create());

    while(simulation->run());

    return 0;
}

