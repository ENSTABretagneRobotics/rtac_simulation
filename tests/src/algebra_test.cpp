#include <iostream>
using namespace std;

#include <rtac_simulation/geometry.h>
using namespace rtac::simulation;

int main()
{
    auto p = DevicePose<float>::Identity();
    p.rotation_matrix() = Eigen::Matrix3<float>::Random();
    p.sanities();
    p.translation() = float3{1,2,3};

    cout << "Pose :\n" << p << endl;
    cout << "Inverse :\n" << p.inverse() << endl;
    cout << "Product :\n" << p*p.inverse() << endl;

    auto p1 = DevicePose<float>::Identity();
    p1.rotation_matrix()(0,0) =  0;
    p1.rotation_matrix()(1,1) =  0;
    p1.rotation_matrix()(1,0) =  1;
    p1.rotation_matrix()(0,1) = -1;

    cout << p1.to_world_frame(float3{1,1,0}) << endl;
    cout << p1.to_local_frame(float3{1,1,0}) << endl;

    cout << p1 * float3{1,1,0} << endl;

    return 0;
}
