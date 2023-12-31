#ifndef _DEF_RTAC_SIMULATION_SENSOR_INSTANCE_H_
#define _DEF_RTAC_SIMULATION_SENSOR_INSTANCE_H_

#include <memory>

#include <rtac_base/types/TypeInfo.h>
#include <rtac_base/types/Linspace.h>
#include <rtac_base/types/Pose.h>
#include <rtac_base/containers/Image.h>
#include <rtac_base/cuda/Texture2D.h>

#include <rtac_simulation/SensorInfo.h>
#include <rtac_simulation/Waveform.h>
#include <rtac_simulation/Binner.h>
#include <rtac_simulation/ReductionKernel.h>

#include <rtac_simulation/Receiver.h>

namespace rtac { namespace simulation {

class SensorInstance : public std::enable_shared_from_this<SensorInstance>
{
    public:

    using Ptr      = std::shared_ptr<SensorInstance>;
    using ConstPtr = std::shared_ptr<const SensorInstance>;
    using Pose     = rtac::Pose<float>;

    protected:

    Linspace<float> ranges_;
    Waveform::Ptr   waveform_;
    Pose            pose_;
    float           soundCelerity_;
    Binner          binner_;

    SensorInstance(const SensorInfo::ConstPtr& info,
                   const Pose& pose,
                   float soundCelerity);

    virtual void  generate_psf_data() = 0;
    virtual void* sample_pointer() = 0;

    public:

    Ptr      ptr()       { return this->shared_from_this(); }
    ConstPtr ptr() const { return this->shared_from_this(); }

    const Pose& pose() const { return pose_; }
          Pose& pose()       { return pose_; }
    const Linspace<float>& ranges() const { return ranges_; }
    Waveform::ConstPtr waveform() const { return waveform_; }
    float sound_celerity() const { return soundCelerity_; }
    Directivity::ConstPtr directivity() const { return this->info().directivity(); }

    void set_ranges(const Linspace<float>& ranges, float soundCelerity);
    void set_ranges(const Linspace<float>& ranges);
    void set_ranges(float maxRange, unsigned int rangeCount);

    ReceiverViewBase receiver_view();

    virtual const SensorInfo& info() const = 0;
    virtual void set_sample_count(unsigned int count) = 0;
    virtual unsigned int sample_count() const = 0;
    virtual bool is_complex() const = 0;
    virtual ScalarId scalar_type() const = 0;
    virtual void compute_output() = 0;

    template <typename T> bool is_type() const;
    template <typename T> std::shared_ptr<const T> safe_cast() const;
    template <typename T> std::shared_ptr<T>       safe_cast();
};

template <typename T>
bool SensorInstance::is_type() const
{
    return std::dynamic_pointer_cast<const T>(this->ptr()) != nullptr;
}

template <typename T>
std::shared_ptr<const T> SensorInstance::safe_cast() const
{ 
    auto ptr = std::dynamic_pointer_cast<const T>(this->ptr());
    if(!ptr) {
        throw TypeError() << " : wrong pointer conversion";
    }
    return ptr;
}

template <typename T>
std::shared_ptr<T> SensorInstance::safe_cast()
{ 
    auto ptr = std::dynamic_pointer_cast<T>(this->ptr());
    if(!ptr) {
        throw TypeError() << " : wrong pointer conversion";
    }
    return ptr;
}


} //namespace simulation
} //namespace rtac

#include <rtac_simulation/SensorInstance2D.h>

#endif //_DEF_RTAC_SIMULATION_SENSOR_INSTANCE_H_
