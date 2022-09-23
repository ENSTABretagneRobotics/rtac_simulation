#ifndef _DEF_RTAC_SIMULATION_POLAR_RAY_CASTER_H_
#define _DEF_RTAC_SIMULATION_POLAR_RAY_CASTER_H_

#include <iostream>
#include <memory>

#include <rtac_base/cuda/DeviceObject.h>

#include <rtac_optix/RaytypeFactory.h>
#include <rtac_optix/Context.h>
#include <rtac_optix/Pipeline.h>
#include <rtac_optix/Material.h>
#include <rtac_optix/ShaderBinding.h>
#include <rtac_optix/ShaderBindingTable.h>

#include <rtac_simulation/Sample.h>
#include <rtac_simulation/Emitter.h>
#include <rtac_simulation/PolarReceiver2D.h>

namespace rtac { namespace simulation {

class PolarRayCaster
{
    public:

    using Ptr      = std::shared_ptr<PolarRayCaster>;
    using ConstPtr = std::shared_ptr<const PolarRayCaster>;

    using Raytypes = optix::RaytypeFactory<Sample3D<float>>;
    using SonarRay = Raytypes::Raytype<0>;
    using SonarMissMaterial = optix::Material<SonarRay, void>;

    struct Params {
        OptixTraversableHandle             objectTree;
        EmitterView<float>                 emitter;
        ReceiverView<PolarSample2D<float>> receiver;
        const float3*                      directions;
        float3*                            outputPoints;
    };

    protected:

    mutable optix::Context::Ptr    context_;
    mutable optix::Pipeline::Ptr   pipeline_;
    optix::Module::Ptr             pathTracingModule_;
    optix::ProgramGroup::Ptr       raygenProgram_;
    optix::ProgramGroup::Ptr       missProgram_;
    optix::ShaderBindingTable::Ptr sbt_;
    optix::GroupInstance::Ptr      objectTree_;

    PolarRayCaster();

    public:

    static Ptr Create() { return Ptr(new PolarRayCaster()); }

    optix::Context::Ptr  context()  const { return context_;  }
    optix::Pipeline::Ptr pipeline() const { return pipeline_; }

    optix::ShaderBindingTable::Ptr sbt() { return sbt_; }
    optix::GroupInstance::Ptr object_tree() { return objectTree_; }

    void trace(Emitter<float>::Ptr              emitter,
              PolarReceiver2D<float>::Ptr       receiver,
              const cuda::DeviceVector<float3>& directions,
              cuda::DeviceVector<float3>&       outputPoints);
};

} //namespace simulation
} //namespace rtac



#endif //_DEF_RTAC_SIMULATION_POLAR_RAY_CASTER_H_
