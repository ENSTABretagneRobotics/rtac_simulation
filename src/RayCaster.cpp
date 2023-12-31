#include <rtac_simulation/RayCaster.h>

#include <rtac_simulation/ptx_files.h>

namespace rtac { namespace simulation {

RayCaster::RayCaster(const Emitter::Ptr& emitter,
                     const SensorInstance::Ptr& receiver) :
    emitter_(emitter),
    receiver_(receiver),
    context_(optix::Context::Create()),
    pipeline_(optix::Pipeline::Create(context_)),
    sbt_(optix::ShaderBindingTable::Create(Raytypes::RaytypeCount)),
    objectTree_(optix::GroupInstance::Create(context_)),
    soundCelerity_(1500.0f)
{
    pipeline_->compile_options().numPayloadValues = 8;
    pipeline_->compile_options().exceptionFlags   = OPTIX_EXCEPTION_FLAG_DEBUG;
    
    auto ptxFiles = rtac_simulation::get_ptx_files();
    pathTracingModule_ = pipeline_->add_module("RayCaster", 
                                               ptxFiles["src/RayCaster.cu"]);

    if(std::dynamic_pointer_cast<SensorInstance2D>(receiver_))
    {
        raygenProgram_ = pipeline_->add_raygen_program("__raygen__polar_ray_caster_2d", 
                                                       "RayCaster");
    }
    else
    {
        raygenProgram_ = pipeline_->add_raygen_program("__raygen__polar_ray_caster_1d", 
                                                       "RayCaster");
    }
    missProgram_ = pipeline_->add_miss_program("__miss__polar_ray_caster",
                                               "RayCaster");

    defaultHitProgram_ = pipeline_->add_hit_programs();
    defaultHitProgram_->set_closesthit({"__closesthit__ray_caster_default_hit",
                                        pipeline_->module("RayCaster")});
    defaultMaterial_ = DefaultMaterial::Create(defaultHitProgram_);

    
    sbt_->set_raygen_record(optix::ShaderBinding<void>::Create(raygenProgram_));
    sbt_->add_miss_record(SonarMissMaterial::Create(missProgram_));
}

RayCaster::Ptr RayCaster::Create(const Emitter::Ptr& emitter,
                                 const SensorInstance::Ptr& receiver)
{
    return Ptr(new RayCaster(emitter, receiver));
}

void RayCaster::trace(cuda::CudaVector<float3>& outputPoints)
{
    cuda::DeviceObject<Params> params;

    outputPoints.resize(emitter_->size());
    receiver_->set_sample_count(emitter_->size());

    params.objectTree    = *objectTree_;
    params.emitter       = emitter_->view();
    params.receiver      = receiver_->receiver_view();
    params.outputPoints  = outputPoints.data();
    params.soundCelerity = soundCelerity_;

    params.update_device();

    OPTIX_CHECK( optixLaunch(*pipeline_, 0,
                             (CUdeviceptr)params.device_ptr(), sizeof(Params),
                             sbt_->sbt(),
                             emitter_->size(), 1, 1) );
    cudaDeviceSynchronize();
    CUDA_CHECK_LAST();
}

void RayCaster::trace(const Emitter&    emitter,
                      SensorInstance2D& receiver,
                      cuda::CudaVector<float3>& outputPoints)
{
    cuda::DeviceObject<Params> params;

    outputPoints.resize(emitter.size());
    receiver.set_sample_count(emitter.size());

    params.objectTree    = *objectTree_;
    params.emitter       = emitter.view();
    params.receiver      = receiver.receiver_view();
    params.outputPoints  = outputPoints.data();
    params.soundCelerity = soundCelerity_;

    params.update_device();

    OPTIX_CHECK( optixLaunch(*pipeline_, 0,
                             (CUdeviceptr)params.device_ptr(), sizeof(Params),
                             sbt_->sbt(),
                             emitter.size(), 1, 1) );
    cudaDeviceSynchronize();
    CUDA_CHECK_LAST();
}

optix::ObjectInstance::Ptr RayCaster::add_object(const cuda::DeviceMesh<>::ConstPtr& mesh)
{
    auto geom = optix::MeshGeometry::Create(context_, mesh);
    geom->material_hit_setup({OPTIX_GEOMETRY_FLAG_NONE});
    geom->enable_vertex_access();

    auto object = optix::ObjectInstance::Create(geom);
    object->add_material(defaultMaterial_);

    objectTree_->add_instance(object);
    sbt_->add_object(object);

    object->build();
    geom->clear_mesh();

    return object;
}

} //namespace simulation
} //namespace rtac


