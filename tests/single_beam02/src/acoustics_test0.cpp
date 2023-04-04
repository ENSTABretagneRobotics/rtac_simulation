#include <iostream>
#include <iomanip>
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

#include <rtac_base/external/nmea_utils.h>
using namespace rtac::nmea;

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

std::vector<rtac::Pose<float>> poses_from_nmea(std::istream& is);
std::vector<rtac::Pose<float>> poses_from_nmea(std::istream& is, double scaling, double x0, double y0);
void write_to_file(const rtac::HostVector<rtac::Complex<float>>& data, std::ofstream& f);

int main()
{
    auto dtmPath = find_one(".*ea400_rade/bathy/.*_WGS84.ply");
    cout << "DTM path : " << dtmPath << endl;

    auto posePath = find_one(".*ea400_rade/.*_nmea.txt");
    cout << "pose path : " << posePath << endl;

    auto simulation = OptixSimulation1::Create("EA400_200KHz_emitter.yaml",
                                               "EA400_200KHz_receiver.yaml");
    //auto simulation = OptixSimulation1::Create("EA400_38KHz_emitter.yaml",
    //                                           "EA400_38KHz_receiver.yaml");
    auto ea400 = std::dynamic_pointer_cast<SensorInstance1D_Complex>(simulation->receiver().ptr());

    auto mesh = HostMesh<>::from_ply(dtmPath);
    auto p0 = mesh->points()[0];
    float scaling = M_PI / 180.0 * 6.371e6;
    for(auto& p : mesh->points()) {
        p = float3{scaling*((double)p.x - (double)p0.x),
                   scaling*((double)p.y - (double)p0.y),
                   p.z};
    }
    std::vector<rtac::Pose<float>> poses;
    {
        auto f = std::ifstream(posePath);
        if(!f) {
            throw std::runtime_error("Could not open pose file");
        }
        poses = poses_from_nmea(f, scaling, p0.x, p0.y);
        auto R = rtac::Pose<float>::from_rotation_matrix(
            Eigen::AngleAxisf(0.5f*M_PI, Eigen::Vector3f::UnitY()).toRotationMatrix());
        for(auto& p : poses) {
            p *= R;
        }
    }

    cout << "Number of points : " << mesh->points().size() << endl;
    cout << "Number of faces  : " << mesh->faces().size() << endl;
    simulation->add_object(DeviceMesh<>::Create(*mesh));

    //DeviceVector<float3> optixPoints;

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

    //plt::Display sonarDisplay(display.context());
    //sonarDisplay.disable_frame_counter();
    //auto pingRenderer = sonarDisplay.create_renderer<plt::OculusRenderer>(plt::View::Create());

    plt::Display simDisplay(display.context());
    simDisplay.disable_frame_counter();
    auto simRenderer = simDisplay.create_renderer<plt::SimplePlot>(plt::View::Create());

    std::ofstream f("out.csv", std::ofstream::out);
    if(!f) {
        throw std::runtime_error("could not open output file");
    }

    auto it = poses.cbegin();
    while(!display.should_close()) {

        simulation->emitter().pose()  = *it;
        simulation->receiver().pose() = *it;
        //simulation->receiver().set_ranges(26.0, 4197);
        //simulation->receiver().set_ranges(25.0, 4096);
        simulation->run();

        write_to_file(ea400->output(), f);

        //auto tmp1 = real(ea400->output());
        auto tmp1 = abs(ea400->output());
        simRenderer->set_data(tmp1);

        optixRenderer->points().copy_from_cuda(simulation->hit_points().size(),
            reinterpret_cast<const typename plt::GLMesh::Point*>(simulation->hit_points().data()));
        trace->add_pose(*it);

        display.draw();
        simDisplay.draw();
        if(++it == poses.cend()) {
            break;
            it = poses.cbegin();
        }
    }

    return 0;
}

std::vector<rtac::Pose<float>> poses_from_nmea(std::istream& is)
{
    std::vector<rtac::Pose<float>> poses;

    std::string line;
    while(std::getline(is, line, '\n')) {
        if(nmea_type(line) != "GPGGA") continue;
        std::cout << line << std::endl;
        poses.push_back(pose_from_gpgga(line, false));
    }
    return poses;
}

std::vector<rtac::Pose<float>> poses_from_nmea(std::istream& is, double scaling, double x0, double y0)
{
    std::vector<rtac::Pose<float>> poses;

    std::string line;
    while(std::getline(is, line, '\n')) {
        if(nmea_type(line) != "GPGGA") continue;
        auto latlonalt = latlonalt_from_gpgga(line, false);
        auto p = rtac::Pose<float>::Identity();
        p.x() = scaling*(latlonalt[0] - x0);
        p.y() = scaling*(latlonalt[1] - y0);
        p.z() = latlonalt[2];
        poses.push_back(p);
    }
    return poses;
}

void write_to_file(const rtac::HostVector<rtac::Complex<float>>& data, std::ofstream& f)
{
    std::ostringstream oss;
    oss << data[0].real() << ' ' << data[0].imag();
    for(unsigned int i = 1; i < data.size(); i++) {
        oss << ' ' << data[i].real() << ' ' << data[i].imag();
    }
    oss << '\n';
    f << oss.str();
}
