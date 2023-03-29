#pragma once

#include <rtac_base/cuda_defines.h>
#include <rtac_base/cuda/utils.h>

#include <rtac_optix/Material.h>

#include <rtac_simulation/RayCaster.h>

namespace rtac { namespace simulation {

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

using SonarMaterial = rtac::optix::Material<RayCaster::SonarRay, PhaseData>;

} //namespace simulation
} //namespace rtac
