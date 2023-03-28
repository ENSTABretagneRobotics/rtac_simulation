#ifndef _DEF_RTAC_SIMULATION_SENSOR_INFO_H_
#define _DEF_RTAC_SIMULATION_SENSOR_INFO_H_

#include <vector>

#include <rtac_base/types/Complex.h>
#include <rtac_base/types/Bounds.h>
#include <rtac_base/types/Linspace.h>
#include <rtac_base/cuda/TextureVector.h>

#include <rtac_simulation/Waveform.h>
#include <rtac_simulation/BeamDirectivity.h>
#include <rtac_simulation/Directivity.h>

namespace rtac { namespace simulation {

class SensorInfo
{
    public:

    using Ptr      = std::shared_ptr<SensorInfo>;
    using ConstPtr = std::shared_ptr<const SensorInfo>;

    protected:

    Linspace<float>  ranges_;
    Waveform::Ptr    waveform_;
    Directivity::Ptr directivity_;
    float soundCelerity_;

    SensorInfo(const Linspace<float>&  ranges,
               const Waveform::Ptr&    waveform,
               const Directivity::Ptr& directivity) :
        ranges_(ranges),
        waveform_(waveform),
        directivity_(directivity),
        soundCelerity_(1500.0f)
    {
        waveform_->set_duration(2*ranges_.resolution() / soundCelerity_);
    }

    public:

    virtual ~SensorInfo() = default; // to be polymorphic

    static Ptr Create(const Linspace<float>&  ranges,
                      const Waveform::Ptr&    waveform,
                      const Directivity::Ptr& directivity)
    {
        return Ptr(new SensorInfo(ranges, waveform, directivity));
    }

    const Linspace<float>& ranges()      const { return ranges_;      }
    Directivity::ConstPtr  directivity() const { return directivity_; }
    Waveform::ConstPtr     waveform()    const { return waveform_;    }
};

class SensorInfo2D : public SensorInfo
{
    public:

    using Ptr      = std::shared_ptr<SensorInfo2D>;
    using ConstPtr = std::shared_ptr<const SensorInfo2D>;

    protected:

    std::vector<float>   bearings_;
    BeamDirectivity::Ptr beamDirectivity_;

    cuda::TextureVector<float> bearingsDeviceData_;

    SensorInfo2D(const std::vector<float>&   bearings,
                 const Linspace<float>&      ranges,
                 const Waveform::Ptr&        waveform,
                 const BeamDirectivity::Ptr& beamDirectivity,
                 const Directivity::Ptr& directivity) :
        SensorInfo(ranges, waveform, directivity),
        bearings_(bearings),
        beamDirectivity_(beamDirectivity),
        bearingsDeviceData_(bearings)
    {}

    public:

    static Ptr Create(const std::vector<float>&   bearings,
                      const Linspace<float>&      ranges,
                      const Waveform::Ptr&        waveform,
                      const BeamDirectivity::Ptr& beamDirectivity,
                      const Directivity::Ptr& directivity)
    {
        return Ptr(new SensorInfo2D(bearings, ranges, waveform, beamDirectivity, directivity));
    }

    unsigned int width()  const { return bearings_.size(); }
    unsigned int height() const { return ranges_.size();   }
    unsigned int size()   const { return this->width()*this->height(); }

    const std::vector<float>& bearings() const { return bearings_; }
    cuda::TextureVectorView<float> bearings_view() const {
        return bearingsDeviceData_.view();
    }

    BeamDirectivity::ConstPtr beam_directivity() const { return beamDirectivity_; }
};


} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_SENSOR_INFO_H_
