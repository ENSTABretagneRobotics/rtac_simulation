#include <rtac_simulation/RayCaster.h>

#include <rtac_base/types/PODWrapper.h>
#include <rtac_base/cuda/geometry.h>

using namespace rtac::simulation;

/**
 * Optix needs C linkage
 */
extern "C" {

//__constant__ PolarRayCaster::Params params;
__constant__ rtac::PODWrapper<RayCaster::Params> params;


__global__ void __raygen__polar_ray_caster()
{
    auto idx   = optixGetLaunchIndex().x;
    //float3 dir = params->directions[idx];

    RayCaster::SonarRay ray;
    ray.datum = params->emitter.sample_value(idx);

    ray.trace(params->objectTree,
              params->emitter.ray_origin(idx),
              params->emitter.ray_direction(idx));

    auto delta  = ray.position - params->emitter.pose.translation();
    auto range  = length(delta);

    if(range > 30.0f || range < 1.0e-4f) {
        params->outputPoints[idx] = float3({0.0f,0.0f,0.0f});
        params->receiver.set_null_sample(idx);
    }
    else {
        //params->outputPoints[idx] = params->receiver.pose.to_local_frame(ray.position);
        const auto& pose = params->receiver.pose;
        params->outputPoints[idx] = pose.rotation_matrix().transpose()
                                  * (ray.position - pose.translation());

        ray.datum /= range*range; // replace this with full complex multiplication.
        params->receiver.set_sample(idx, ray.datum, range, -delta / range);
        //params->receiver.set_sample(idx, 1.0f, range, -delta / range);
    }
}

__global__ void __miss__polar_ray_caster()
{
    auto res = Sample3D<float>::Zero();
    res.position.x = params->emitter.pose.x();
    res.position.y = params->emitter.pose.y();
    res.position.z = params->emitter.pose.z();
    RayCaster::SonarRay::set_payload(res);
    //RayCaster::SonarRay::set_payload(Sample3D<float>::Zero());
}

};





