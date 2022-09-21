#include "oculus_sim.h"

using namespace narval;

extern "C" {
    __constant__ Params params;
};

extern "C" __global__ void __raygen__oculus_sonar()
{
    auto idx = optixGetLaunchIndex().x;
    float3 dir = params.directions[idx];

    SonarRay ray;
    ray.datum = params.emitter.sample_value(dir);
    
    ray.trace(params.topObject,
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
extern "C" __global__ void __miss__oculus_sonar()
{
    SonarRay::set_payload(rtac::simulation::Sample3D<float>::Zero());
}

extern "C" __global__ void __closesthit__oculus_sonar_phased()
{
    static constexpr const float wavelengthFactor = 4.0f*M_PI / (1500.0f / 1.2e6f);
    static constexpr const float reflectionShift  = 0.5f*M_PI;

    auto phaseData = reinterpret_cast<const PhaseData*>(optixGetSbtDataPointer());

    float3 hitP, hitN;
    helpers::get_triangle_hit_data(hitP, hitN);

    float3 travel = optixTransformPointFromWorldToObjectSpace(optixGetWorldRayOrigin()) - hitP;
    float travelSquared = dot(travel, travel); // replace this with tmax
    float phase = sqrtf(travelSquared) * wavelengthFactor    // phasing due to travel
                + reflectionShift                            // phasing due to reflection
                + phaseData->data[optixGetPrimitiveIndex()]; // phasing random terrain contribution
    
    float a = dot(travel,hitN) / travelSquared;

    auto payload = SonarRay::from_registers();
    payload.datum    *= thrust::complex<float>{a*cos(phase), a*sin(phase)};
    payload.position  = hitP;

    SonarRay::set_payload(payload);
}



