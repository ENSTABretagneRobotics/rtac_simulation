#include "oculus_sim.h"

#include <rtac_optix/helpers/maths.h>

using namespace rtac::simulation;

extern "C" __global__ void __closesthit__oculus_sonar_phased()
{
    static constexpr const float wavelengthFactor = 4.0f*M_PI / (1500.0f / 1.2e6f);
    static constexpr const float reflectionShift  = 0.5f*M_PI;

    auto phaseData = reinterpret_cast<const PhaseData*>(optixGetSbtDataPointer());

    float3 hitP, hitN;
    rtac::optix::helpers::get_triangle_hit_data(hitP, hitN);

    float3 travel = optixTransformPointFromWorldToObjectSpace(optixGetWorldRayOrigin()) - hitP;
    float travelSquared = dot(travel, travel); // replace this with tmax
    float phase = sqrtf(travelSquared) * wavelengthFactor    // phasing due to travel
                + reflectionShift                            // phasing due to reflection
                + phaseData->data[optixGetPrimitiveIndex()]; // phasing random terrain contribution
    
    float a = dot(travel,hitN) / travelSquared;

    auto payload = RayCaster::SonarRay::from_registers();
    payload.datum    *= rtac::Complex<float>{a*cos(phase), a*sin(phase)};
    payload.position  = hitP;

    RayCaster::SonarRay::set_payload(payload);
}



