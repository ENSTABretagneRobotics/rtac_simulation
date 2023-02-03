#include <rtac_simulation/Emitter.h>

namespace rtac { namespace simulation {

Emitter2::Emitter2(const cuda::DeviceVector<float3>& rayDirs,
                   Directivity::ConstPtr directivity,
                   const Pose& pose) :
    Antenna(directivity, pose),
    directions_(rayDirs),
    initialValues_(rayDirs.size())
{
    this->load_initial_values();
}

Emitter2::Ptr Emitter2::Create(const cuda::DeviceVector<float3>& rayDirs,
                               Directivity::ConstPtr directivity,
                               const Pose& pose)
{
    return Ptr(new Emitter2(rayDirs, directivity, pose));
}

Emitter2::Ptr Emitter2::Create(float resolution,
                               float bearingAperture,
                               float elevationAperture,
                               Directivity::ConstPtr directivity,
                               const Pose& pose)
{
    return Create(Emitter<float>::generate_polar_directions(resolution,
                                                            bearingAperture,
                                                            elevationAperture),
                  directivity, pose); 
}

__global__ void load_emitter_initial_values(unsigned int size,
                                            Complex<float>* values,
                                            const float3* directions,
                                            DirectivityView directivity)
{
    auto tid = blockDim.x*blockIdx.x + threadIdx.x;
    if(tid < size) {
        values[tid] = directivity(directions[tid]);
    }
}

void Emitter2::load_initial_values()
{
    load_emitter_initial_values<<<this->size() / 256 + 1, 256>>>(
        this->size(), initialValues_.data(), directions_.data(),
        this->directivity()->view());
    cudaDeviceSynchronize();
    CUDA_CHECK_LAST();
}

} //namespace simulation
} //namespace rtac
