#include <rtac_simulation/SimulationGL.h>

#include <rtac_simulation/factories/utilities.h>
#include <rtac_simulation/factories/EmitterFactory.h>
#include <rtac_simulation/factories/SensorInfoFactory.h>

namespace rtac { namespace simulation {

namespace dsp = rtac::display;

SimulationGL::SimulationGL(const EmitterGL::Ptr& emitter,
                           const SensorInstance::Ptr& receiver) :
    Simulation1(emitter, receiver),
    emitter_(emitter),
    receiver_(receiver)
{
    if(!glfwInit()) { // necessary for glfwWindowHint
        throw std::runtime_error("GLFW initialization failure");
    }
    GLFW_CHECK( glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE) );
    drawSurface_ = std::make_shared<dsp::Display>(emitter->width(), emitter->height());

    frameBuffer_  = dsp::GLFrameBuffer::Create();
    renderTarget_ = dsp::GLRenderBuffer::Create(drawSurface_->window_shape(), GL_RGBA32F);
    depthBuffer_  = dsp::GLRenderBuffer::Create(drawSurface_->window_shape(), GL_DEPTH24_STENCIL8);

    frameBuffer_->bind(GL_FRAMEBUFFER);
    renderTarget_->bind();
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                              GL_RENDERBUFFER, renderTarget_->gl_id());
    depthBuffer_->bind();
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT,
                              GL_RENDERBUFFER, depthBuffer_->gl_id());
    if(!frameBuffer_->is_complete()) {
        throw std::runtime_error("SimulationGL : FBO not complete");
    }
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    rayCaster_ = RayCasterGL::Create(drawSurface_->context());

    casterOutput_.resize(emitter->ray_count());
    receiver->set_sample_count(emitter->ray_count());

    GL_CHECK_LAST();
    // for eventual next windows
    GLFW_CHECK( glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE) );
}

SimulationGL::Ptr SimulationGL::Create(const EmitterGL::Ptr& emitter,
                                       const SensorInstance::Ptr& receiver)
{
    return Ptr(new SimulationGL(emitter, receiver));
}

SimulationGL::Ptr SimulationGL::Create(const EmitterBase::Ptr& emitter,
                                       const SensorInstance::Ptr& receiver)
{
    auto emitterPtr = std::dynamic_pointer_cast<EmitterGL>(emitter);
    if(!emitterPtr) {
        throw std::runtime_error("SimulationGL need to be instanciated with an EmitterGL instance");
    }
    return Create(emitterPtr, receiver);
}

SimulationGL::Ptr SimulationGL::Create(const std::string& emitterFilename,
                                       const std::string& receiverFilename)
{
    auto emitterPath  = FileFinder::FindOne(emitterFilename);
    if(emitterPath == FileFinder::NotFound) {
        throw ConfigError() << " : could not find config file '" << emitterFilename << "'";
    }
    auto receiverPath = FileFinder::FindOne(receiverFilename);
    if(receiverPath == FileFinder::NotFound) {
        throw ConfigError() << " : could not find config file '" << receiverFilename << "'";
    }

    return Create(EmitterFactory::Make(emitterPath),
                  SensorFactory::Make(receiverPath));
}

void SimulationGL::run()
{
    drawSurface_->grab_context();

    // render with OpenGL
    GL_CHECK_LAST();
    frameBuffer_->bind(GL_FRAMEBUFFER);
    GL_CHECK_LAST();
    glClearColor(0,0,0,0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    rayCaster_->draw(emitter_->view());
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    GL_CHECK_LAST();

    //copy framebuffer content to CUDA accessible memory (OpenGL Buffer object)
    frameBuffer_->bind(GL_READ_FRAMEBUFFER);
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    casterOutput_.bind(GL_PIXEL_PACK_BUFFER);
    glReadPixels(0, 0, emitter_->width(), emitter_->height(), GL_RGBA, GL_FLOAT, 0);
    GL_CHECK_LAST();
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);

    this->fill_receiver();
    receiver_->compute_output();

    for(const auto& sink : this->sinks_) {
        sink->set_output(receiver_);
    }
}

__global__ void do_fill_receiver(ReceiverView<SimSample2D> receiver,
                                 ImageView<SimSample2D> samples)
{
    auto h = blockIdx.y;
    auto w = blockDim.x*blockIdx.x + threadIdx.x;
    auto sampleIdx = samples.width()*h + w;
    if(w < samples.width()) {
        //receiver.samples[sampleIdx] = samples(h,w);
        receiver.sample(sampleIdx) = samples(h,w);
    }
}

void SimulationGL::fill_receiver()
{
    //static constexpr unsigned int BlockSize = 256;
    //uint3 grid{emitter_->width() / BlockSize + 1, emitter_->height(), 1};

    //auto receiverPtr = std::dynamic_pointer_cast<SensorInstance2D>(receiver_);
    //if(!receiverPtr) {
    //    throw std::runtime_error("Only SensorInstance2D implemented in SimulationGL for now");
    //}
    //auto samplesPtr  = casterOutput_.map_cuda();
    //do_fill_receiver<<<grid, BlockSize>>>(receiverPtr->receiver_view(),
    //    ImageView<SimSample2D>(emitter_->width(), emitter_->height(), samplesPtr));
    //cudaDeviceSynchronize();
    //CUDA_CHECK_LAST();

    auto receiverPtr = std::dynamic_pointer_cast<SensorInstance2D>(receiver_);
    if(!receiverPtr) {
        throw std::runtime_error("Only SensorInstance2D implemented in SimulationGL for now");
    }

    auto samplesPtr  = casterOutput_.map_cuda();
    cudaMemcpy(receiverPtr->samples().data(), samplesPtr, 
               receiverPtr->samples().size()*sizeof(SimSample2D),
               cudaMemcpyDeviceToDevice);
    CUDA_CHECK_LAST();
}

} //namespace simulation
} //namespace rtac
