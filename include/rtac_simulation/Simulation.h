#ifndef _DEF_RTAC_SIMULATION_SIMULATION_H_
#define _DEF_RTAC_SIMULATION_SIMULATION_H_

#include <memory>
#include <type_traits>

#include <rtac_simulation/Emitter.h>
#include <rtac_simulation/SensorInstance.h>
#include <rtac_simulation/sinks/Sink.h>
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

    template <class T>
    std::shared_ptr<T> add_sink(const std::shared_ptr<T>& sink);

    virtual const EmitterBase&    emitter() const = 0;
    virtual       EmitterBase&    emitter() = 0;
    virtual const SensorInstance& receiver() const = 0;
    virtual       SensorInstance& receiver() = 0;
    virtual bool run() = 0;
};

template <class T> inline
std::shared_ptr<T> Simulation1::add_sink(const std::shared_ptr<T>& sink)
{
    static_assert(std::is_base_of<Sink,T>::value, "Type T should be derived from Sink type");
    sinks_.push_back(sink);
    return sink;
}

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_SIMULATION_H_

