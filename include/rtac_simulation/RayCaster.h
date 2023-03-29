#ifndef _DEF_RTAC_SIMULATION_RAY_CASTER_H_
#define _DEF_RTAC_SIMULATION_RAY_CASTER_H_

#include <iostream>
#include <memory>

#include <rtac_base/cuda/DeviceObject.h>
#include <rtac_base/cuda/DeviceMesh.h>

#include <rtac_optix/RaytypeFactory.h>
#include <rtac_optix/Context.h>
#include <rtac_optix/Pipeline.h>
#include <rtac_optix/Material.h>
#include <rtac_optix/ShaderBinding.h>
#include <rtac_optix/ShaderBindingTable.h>
#include <rtac_optix/MeshGeometry.h>
#include <rtac_optix/ObjectInstance.h>

#include <rtac_simulation/Sample.h>
#include <rtac_simulation/Emitter.h>
#include <rtac_simulation/Receiver.h>
#include <rtac_simulation/SensorInstance.h>

namespace rtac { namespace simulation {

class RayCaster : public std::enable_shared_from_this<RayCaster>
{
    public:

    using Ptr      = std::shared_ptr<RayCaster>;
    using ConstPtr = std::shared_ptr<const RayCaster>;

    //using Raytypes = optix::RaytypeFactory<Sample3D<float>>;
    using Raytypes = optix::RaytypeFactory<RayPayload>;
    using SonarRay = Raytypes::Raytype<0>;
    using SonarMissMaterial = optix::Material<SonarRay, void>;
    using DefaultMaterial   = optix::Material<SonarRay, void>;

    struct Params {
        OptixTraversableHandle    objectTree;
        EmitterView               emitter;
        ReceiverViewBase          receiver;
        //const float3*              directions;
        float3*                   outputPoints;
        float soundCelerity;
    };

    protected:

    Emitter::Ptr        emitter_;
    SensorInstance::Ptr receiver_;

    mutable optix::Context::Ptr    context_;
    mutable optix::Pipeline::Ptr   pipeline_;
    optix::Module::Ptr             pathTracingModule_;
    optix::ProgramGroup::Ptr       raygenProgram_;
    optix::ProgramGroup::Ptr       missProgram_;
    optix::ShaderBindingTable::Ptr sbt_;
    optix::GroupInstance::Ptr      objectTree_;

    optix::ProgramGroup::Ptr       defaultHitProgram_;
    DefaultMaterial::Ptr           defaultMaterial_;

    float soundCelerity_;

    RayCaster(const Emitter::Ptr& emitter,
              const SensorInstance::Ptr& receiver);

    public:

    static Ptr Create(const Emitter::Ptr& emitter,
                      const SensorInstance::Ptr& receiver);

    Ptr      ptr()       { return this->shared_from_this(); }
    ConstPtr ptr() const { return this->shared_from_this(); }

    const EmitterBase&    emitter()  const { return *emitter_; }
          EmitterBase&    emitter()        { return *emitter_; }
    const SensorInstance& receiver() const { return *receiver_; }
          SensorInstance& receiver()       { return *receiver_; }

    optix::Context::Ptr  context()  const { return context_;  }
    optix::Pipeline::Ptr pipeline() const { return pipeline_; }

    optix::ShaderBindingTable::Ptr sbt() { return sbt_; }
    optix::GroupInstance::Ptr object_tree() { return objectTree_; }

    const DefaultMaterial::Ptr& default_material() { return defaultMaterial_; }
    optix::ObjectInstance::Ptr add_object(const cuda::DeviceMesh<>::ConstPtr& mesh);

    void trace(cuda::DeviceVector<float3>& outputPoints);

    void trace(Emitter::Ptr                emitter,
               SensorInstance2D::Ptr       receiver,
               cuda::DeviceVector<float3>& outputPoints)
    {
        this->trace(*emitter, *receiver, outputPoints);
    }
    void trace(const Emitter&    emitter,
               SensorInstance2D& receiver,
               cuda::DeviceVector<float3>& outputPoints);

    float sound_celerity() const      { return soundCelerity_; }
    void  set_sound_celerity(float c) { soundCelerity_ = c;    }
};

} //namespace simulation
} //namespace rtac



#endif //_DEF_RTAC_SIMULATION_RAY_CASTER_H_
