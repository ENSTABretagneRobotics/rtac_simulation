#include <rtac_simulation/PolarRayCaster.h>

using namespace rtac::simulation;

/**
 * Optix needs C linkage
 */
extern "C" {

__constant__ PolarRayCaster::Params params;


__global__ void __raygen__polar_ray_caster()
{
    auto idx   = optixGetLaunchIndex().x;
    float3 dir = params.directions[idx];

    PolarRayCaster::SonarRay ray;
    ray.datum = params.emitter.sample_value(dir);

    ray.trace(params.objectTree,
              params.emitter.pose.translation(),
              params.emitter.ray_direction(dir));

    auto delta  = ray.position - params.emitter.pose.translation();
    auto range  = length(delta);

    if(range > 30.0f || range < 1.0e-6f) {
        params.outputPoints[idx]  = float3({0.0f,0.0f,0.0f});
        params.receiver.samples[idx] = rtac::simulation::PolarSample2D<float>::Zero();
    }
    else {
        params.outputPoints[idx] = params.receiver.pose.to_local_frame(ray.position);

        ray.datum /= range*range; // replace this with full complex multiplication.
        params.receiver.set_sample(idx, ray);
    }
}

__global__ void __miss__polar_ray_caster()
{
    PolarRayCaster::SonarRay::set_payload(Sample3D<float>::Zero());
}

};





