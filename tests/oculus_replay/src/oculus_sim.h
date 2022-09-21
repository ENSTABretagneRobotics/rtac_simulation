#pragma once

#include <rtac_base/types/common.h>

#include <rtac_optix/RaytypeFactory.h>
#include <rtac_optix/Material.h>

#include <rtac_simulation/Sample.h>
#include <rtac_simulation/Emitter.h>
#include <rtac_simulation/Receiver.h>
#include <rtac_simulation/examples/blueprint_oculus.h>

namespace narval {

using namespace rtac;
using namespace rtac::cuda;
using namespace rtac::optix;

struct PhaseData
{
    // this is an array of randow phases [-180, 180] assigned to each dtm
    // points at initialization.
    size_t size;
    const float* data; 

    RTAC_HOSTDEVICE float operator[](unsigned int idx) const {
        return data[idx];
    }
};

using Raytypes          = RaytypeFactory<rtac::simulation::Sample3D<float>>;
using SonarRay          = Raytypes::Raytype<0>;
using SonarMaterial     = Material<SonarRay, PhaseData>;
using SonarMissMaterial = Material<SonarRay, void>;

struct Params {
    OptixTraversableHandle topObject;
    float3*  outputPoints;
    rtac::simulation::EmitterView<float> emitter;
    rtac::simulation::ReceiverView<rtac::simulation::PolarSample2D<float>> receiver;
    float3*     directions;
};

}; //namespace narval

