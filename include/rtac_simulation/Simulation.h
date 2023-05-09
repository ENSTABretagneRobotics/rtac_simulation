#ifndef _DEF_RTAC_SIMULATION_SIMULATION_H_
#define _DEF_RTAC_SIMULATION_SIMULATION_H_

#include <memory>

#include <rtac_simulation/Emitter.h>
#include <rtac_simulation/SensorInstance.h>
#include <rtac_simulation/Sink.h>
#include <rtac_simulation/PoseSource.h>
#include <rtac_simulation/factories/utilities.h>

namespace rtac { namespace simulation {

class Simulation1
{
    public:

    using Ptr      = std::shared_ptr<Simulation1>;
    using ConstPtr = std::shared_ptr<const Simulation1>;

    protected:

    std::deque<Sink::Ptr> sinks_;
    PoseSource::Ptr emitterPoses_;
    PoseSource::Ptr receiverPoses_;

    Simulation1(const EmitterBase::Ptr&, const SensorInstance::Ptr&) {}

    //virtual EmitterBase::Ptr    emitter_ptr()  = 0;
    //virtual SensorInstance::Ptr receiver_ptr() = 0;

    public:

    void set_emitter_pose_source(const PoseSource::Ptr& poses)  { emitterPoses_  = poses; }
    void set_receiver_pose_source(const PoseSource::Ptr& poses) { receiverPoses_ = poses; }

    Sink::Ptr add_sink(const Sink::Ptr& sink) { 
        sinks_.push_back(sink);
        return sink;
    }

    virtual const EmitterBase&    emitter() const = 0;
    virtual       EmitterBase&    emitter() = 0;
    virtual const SensorInstance& receiver() const = 0;
    virtual       SensorInstance& receiver() = 0;
    virtual bool run() = 0;
};

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_SIMULATION_H_

