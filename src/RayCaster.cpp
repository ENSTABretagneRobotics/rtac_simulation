#include <rtac_simulation/RayCaster.h>

#include <rtac_simulation/ptx_files.h>

namespace rtac { namespace simulation {

RayCaster::RayCaster() :
    context_(optix::Context::Create()),
    pipeline_(optix::Pipeline::Create(context_)),
    sbt_(optix::ShaderBindingTable::Create(Raytypes::RaytypeCount)),
    objectTree_(optix::GroupInstance::Create(context_))
{
    pipeline_->compile_options().numPayloadValues = 8;
    pipeline_->compile_options().exceptionFlags   = OPTIX_EXCEPTION_FLAG_DEBUG;
    
    auto ptxFiles = rtac_simulation::get_ptx_files();
    pathTracingModule_ = pipeline_->add_module("RayCaster", 
                                               ptxFiles["src/RayCaster.cu"]);

    raygenProgram_ = pipeline_->add_raygen_program("__raygen__polar_ray_caster", 
                                                   "RayCaster");
    missProgram_ = pipeline_->add_miss_program("__miss__polar_ray_caster",
                                               "RayCaster");

    sbt_->set_raygen_record(optix::ShaderBinding<void>::Create(raygenProgram_));
    sbt_->add_miss_record(SonarMissMaterial::Create(missProgram_));
}

void RayCaster::trace(Emitter2::Ptr                     emitter,
                      Receiver2<SimSample2D>::Ptr       receiver,
                      //const cuda::DeviceVector<float3>& directions,
                      cuda::DeviceVector<float3>&       outputPoints)
{
    cuda::DeviceObject<Params> params;

    outputPoints.resize(emitter->size());

    params.objectTree   = *objectTree_;
    params.emitter      = emitter->view();
    params.receiver     = receiver->view();
    //params.directions   = directions.data();
    params.outputPoints = outputPoints.data();

    params.update_device();

    OPTIX_CHECK( optixLaunch(*pipeline_, 0,
                             (CUdeviceptr)params.device_ptr(), sizeof(Params),
                             sbt_->sbt(),
                             emitter->size(), 1, 1) );
    cudaDeviceSynchronize();
    CUDA_CHECK_LAST();
}

void RayCaster::trace(Emitter2::Ptr               emitter,
                      SensorInstance2D::Ptr       receiver,
                      cuda::DeviceVector<float3>& outputPoints)
{
    cuda::DeviceObject<Params> params;

    outputPoints.resize(emitter->size());
    receiver->set_sample_count(emitter->size());

    params.objectTree   = *objectTree_;
    params.emitter      = emitter->view();
    params.receiver     = receiver->receiver_view();
    params.outputPoints = outputPoints.data();

    params.update_device();

    OPTIX_CHECK( optixLaunch(*pipeline_, 0,
                             (CUdeviceptr)params.device_ptr(), sizeof(Params),
                             sbt_->sbt(),
                             emitter->size(), 1, 1) );
    cudaDeviceSynchronize();
    CUDA_CHECK_LAST();
}

} //namespace simulation
} //namespace rtac


