#ifndef _DEF_RTAC_SIMULATION_SINK_H_
#define _DEF_RTAC_SIMULATION_SINK_H_

#include <memory>

#include <rtac_simulation/SensorInstance.h>

namespace rtac { namespace simulation {

/**
 * Base class to Handle simulated data after generation
 */
class Sink
{
    public:

    using Ptr      = std::shared_ptr<Sink>;
    using ConstPtr = std::shared_ptr<const Sink>;

    protected:

    Sink() {}

    public:

    virtual ~Sink() = default;

    virtual void set_output(const SensorInstance::Ptr& sensor) = 0;
};

/**
 * This Sink does nothing. For place holding purpuses.
 */
class SinkTrash : public Sink
{
    public:

    using Ptr      = std::shared_ptr<SinkTrash>;
    using ConstPtr = std::shared_ptr<const SinkTrash>;

    protected:

    SinkTrash() {}

    public:

    void set_output(const SensorInstance::Ptr& sensor) override {}
};

} //namespace simulation
} //namespace rtac


#endif //_DEF_RTAC_SIMULATION_SINK_H_
