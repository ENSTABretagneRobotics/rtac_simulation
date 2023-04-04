#include <rtac_simulation/Emitter.h>

#include <rtac_base/types/Mesh.h>

namespace rtac { namespace simulation {

Emitter::Emitter(const cuda::DeviceVector<float3>& rayDirs,
                 Directivity::ConstPtr directivity,
                 float frequency, const Pose& pose) :
    EmitterBase(frequency, pose),
    directivity_(directivity),
    directions_(rayDirs),
    initialValues_(rayDirs.size())
{
    this->load_initial_values();
}

Emitter::Ptr Emitter::Create(const cuda::DeviceVector<float3>& rayDirs,
                             Directivity::ConstPtr directivity,
                             float frequency, const Pose& pose)
{
    return Ptr(new Emitter(rayDirs, directivity, frequency, pose));
}

Emitter::Ptr Emitter::Create(float resolution,
                             float bearingAperture,
                             float elevationAperture,
                             Directivity::ConstPtr directivity,
                             float frequency, const Pose& pose)
{
    return Create(generate_polar_directions(resolution,
                                            bearingAperture,
                                            elevationAperture),
                  directivity, frequency, pose); 
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

void Emitter::load_initial_values()
{
    load_emitter_initial_values<<<this->size() / 256 + 1, 256>>>(
        this->size(), initialValues_.data(), directions_.data(),
        this->directivity()->view());
    cudaDeviceSynchronize();
    CUDA_CHECK_LAST();
}

cuda::DeviceVector<float3> Emitter::generate_polar_directions(float resolution,
                                                              float bearingAperture,
                                                              float elevationAperture)
{
    resolution        *= M_PI  / 180.0f;
    bearingAperture   *= M_PI  / 180.0f;
    elevationAperture *= M_PI  / 180.0f;

    //unsigned int idx = 0;
    unsigned int H   = (int)(elevationAperture / resolution) + 1;
    
    std::vector<float3> data;
    for(int h = 0; h < H; h++) {
        float phi = elevationAperture * (((float)h) / (H - 1) - 0.5f);
        int W = (int)(abs(std::cos(phi)) * bearingAperture   / resolution) + 1;
        for(int w = 0; w < W; w++) {
            float theta = bearingAperture * (((float)w) / (W - 1) - 0.5f);
            data.push_back(float3{std::cos(theta)*std::cos(phi),
                                  std::sin(theta)*std::cos(phi),
                                  std::sin(phi)});
        }
    }
    return cuda::DeviceVector<float3>(data);
}

cuda::DeviceVector<float3>
    Emitter::generate_icosahedron_directions(float resolution,
                                             float bearingAperture,
                                             float elevationAperture)
{
    unsigned level = std::log2(63.43494882292201 / resolution);
    auto mesh = make_icosphere(level);

    bearingAperture   *= M_PI  / 180.0f;
    elevationAperture *= M_PI  / 180.0f;

    std::vector<float3> dirs;
    for(const auto& p : mesh->points()) {
        if(std::abs(std::atan2(p.y, p.x)) < 0.5f*bearingAperture &&
           std::abs(std::atan2(p.z, p.x)) < 0.5f*elevationAperture)
        {
            dirs.push_back(float3{p.x, p.y, p.z});
        }
    }

    return cuda::DeviceVector<float3>(dirs);
}


} //namespace simulation
} //namespace rtac
