#include <iostream>
#include <vector>
using namespace std;

#include <rtac_base/cuda/DeviceVector.h>
#include <rtac_base/cuda/HostVector.h>
using namespace rtac::cuda;

#include <rtac_simulation/WaveField.h>
using namespace rtac::simulation;

constexpr unsigned int BlockSize = 1024;

__global__ void fill(WaveFieldView<float> waves)
{
    auto tid = blockDim.x*blockIdx.x + threadIdx.x;
    for(; tid < waves.size(); tid++) {
        waves.sample(tid) = Complex<float>{0.0f,1.0f};
        waves.x(tid) = 2.0f;
        waves.y(tid) = 3.0f;
        waves.z(tid) = 4.0f;
    }
}

int main()
{
    WaveField<float, DeviceVector> waves(10);
    fill<<<waves.size() / BlockSize + 1, BlockSize>>>(waves.view());
    cudaDeviceSynchronize();
    WaveField<float, HostVector> hwaves(waves);

    for(int i = 0; i < hwaves.size(); i++) {
        cout << hwaves.sample(i)
             << " " << hwaves.x(i)
             << " " << hwaves.y(i)
             << " " << hwaves.z(i) << endl;
    }

    return 0;
}
