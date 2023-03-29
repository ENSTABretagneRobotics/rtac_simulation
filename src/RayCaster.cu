#include <rtac_simulation/RayCaster.h>

#include <rtac_base/types/PODWrapper.h>
#include <rtac_base/cuda/geometry.h>
#include <rtac_optix/helpers/maths.h>

using namespace rtac::simulation;

/**
 * Optix needs C linkage
 */
extern "C" {

//__constant__ PolarRayCaster::Params params;
__constant__ rtac::PODWrapper<RayCaster::Params> params;

__global__ void __raygen__polar_ray_caster_1d()
{
    auto idx   = optixGetLaunchIndex().x;

    RayCaster::SonarRay ray;
    ray.value()  = params->emitter.sample_value(idx);
    ray.travel() = 0.0f;

    auto origin = params->emitter.ray_origin(idx);
    auto dir    = params->emitter.ray_direction(idx);
    ray.trace(params->objectTree, origin, dir);

    //auto distance = ray.travel() * params->soundCelerity;
    auto distance = ray.travel();
    if(distance < 1.0e-4f) {
        params->outputPoints[idx] = float3({0.0f,0.0f,0.0f});
        params->receiver.cast<SimSample1D>().set_null_sample(idx);
    }
    else {
        float phase = (4.0*M_PI*params->emitter.frequency / params->soundCelerity) * distance;
        ray.value() *= rtac::Complex<float>(cos(phase), sin(phase))
                     / (distance*distance);

        params->receiver.cast<SimSample1D>().set_sample(idx, ray.value(), distance, -dir);

        const auto& pose = params->receiver.cast<SimSample1D>().pose;
        params->outputPoints[idx] = pose.rotation_matrix().transpose()
                                  * (distance * dir);
    }
}

__global__ void __raygen__polar_ray_caster_2d()
{
    auto idx   = optixGetLaunchIndex().x;

    RayCaster::SonarRay ray;
    ray.value()  = params->emitter.sample_value(idx);
    ray.travel() = 0.0f;

    auto origin = params->emitter.ray_origin(idx);
    auto dir    = params->emitter.ray_direction(idx);
    ray.trace(params->objectTree, origin, dir);

    //auto distance = ray.travel() * params->soundCelerity;
    auto distance = ray.travel();
    if(distance < 1.0e-4f) {
        params->outputPoints[idx] = float3({0.0f,0.0f,0.0f});
        params->receiver.cast<SimSample2D>().set_null_sample(idx);
    }
    else {
        float phase = (4.0*M_PI*params->emitter.frequency / params->soundCelerity) * distance;
        ray.value() *= rtac::Complex<float>(cos(phase), sin(phase))
                     / (distance*distance);

        params->receiver.cast<SimSample2D>().set_sample(idx, ray.value(), distance, -dir);

        const auto& pose = params->receiver.cast<SimSample2D>().pose;
        params->outputPoints[idx] = pose.rotation_matrix().transpose()
                                  * (distance * dir);
    }
}

__global__ void __miss__polar_ray_caster()
{
    RayCaster::SonarRay::set_payload(RayPayload::Null());
}

//__global__ void __closesthit__ray_caster_default_hit()
//{
//    static constexpr const float wavelengthFactor = 4.0f*M_PI / (1500.0f / 1.2e6f);
//    static constexpr const float reflectionShift  = 0.5f*M_PI;
//
//    float3 hitP, hitN;
//    rtac::optix::helpers::get_triangle_hit_data(hitP, hitN);
//    float3 travel = optixTransformPointFromWorldToObjectSpace(optixGetWorldRayOrigin()) - hitP;
//    float d = optixGetRayTmax(); // travel distance
//
//    float phase = d * wavelengthFactor + reflectionShift;
//    float a = dot(travel,hitN) / (d*d*d);
//    //float a = dot(travel,hitN) / d;
//
//    auto payload = RayCaster::SonarRay::from_registers();
//    payload.datum    *= rtac::Complex<float>{a*cos(phase), a*sin(phase)};
//    payload.position  = hitP;
//
//    RayCaster::SonarRay::set_payload(payload);
//}

__global__ void __closesthit__ray_caster_default_hit()
{
    float3 hitP, hitN;
    rtac::optix::helpers::get_triangle_hit_data(hitP, hitN);
    float3 travel = optixTransformPointFromWorldToObjectSpace(optixGetWorldRayOrigin()) - hitP;
    float d = optixGetRayTmax(); // travel distance

    float a = dot(travel,hitN) / (d*d*d); 
    //float a = dot(travel,hitN) / d;

    auto payload = RayCaster::SonarRay::from_registers();
    payload.value()  *= rtac::Complex<float>(0.0f, a);
    //payload.travel() += d / params->soundCelerity;
    payload.travel() += d;

    RayCaster::SonarRay::set_payload(payload);
}
};





