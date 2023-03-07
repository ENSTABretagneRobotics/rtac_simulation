#ifndef _DEF_RTAC_SIMULATION_SENSOR_INFO_H_
#define _DEF_RTAC_SIMULATION_SENSOR_INFO_H_

#include <vector>

#include <rtac_base/types/Complex.h>
#include <rtac_base/types/Bounds.h>
#include <rtac_base/types/Linspace.h>

#include <rtac_simulation/PointSpreadFunction.h>
#include <rtac_simulation/Directivity.h>

namespace rtac { namespace simulation {

class SensorInfo2D
{
    public:

    using Ptr      = std::shared_ptr<SensorInfo2D>;
    using ConstPtr = std::shared_ptr<const SensorInfo2D>;

    protected:

    std::vector<float>         bearings_;
    Linspace<float>            ranges_;
    PointSpreadFunction2D::Ptr psf_;
    Directivity::Ptr           directivity_;

    SensorInfo2D(const std::vector<float>& bearings,
                 const Linspace<float>&    ranges,
                 const PointSpreadFunction2D::Ptr& psf,
                 const Directivity::Ptr& directivity) :
        bearings_(bearings),
        ranges_(ranges),
        psf_(psf),
        directivity_(directivity)
    {
        psf_->set_pulse_length(2*ranges_.resolution());
    }

    public:

    static Ptr Create(const std::vector<float>& bearings,
                      const Linspace<float>&    ranges,
                      const PointSpreadFunction2D::Ptr& psf,
                      const Directivity::Ptr& directivity)

    {
        return Ptr(new SensorInfo2D(bearings, ranges, psf, directivity));
    }

    unsigned int width()  const { return bearings_.size(); }
    unsigned int height() const { return ranges_.size();   }
    unsigned int size()   const { return this->width()*this->height(); }

    const std::vector<float>& bearings() const { return bearings_; }
    const Linspace<float>&    ranges()   const { return ranges_;   }
    PointSpreadFunction2D::ConstPtr point_spread_function() const { return psf_; }
    Directivity::ConstPtr directivity() const { return directivity_; }

    void set_bearings(const std::vector<float>& bearings) {
        bearings_ = bearings;
    }
    void set_ranges(const Linspace<float>& ranges) {
        ranges_   = ranges;
        psf_->set_pulse_length(2*ranges_.resolution());
    }
    void set_ranges(const Bounds<float>& bounds, unsigned int count) {
        this->set_ranges(Linspace<float>(bounds, count));
    }
    void reconfigure(const std::vector<float>& bearings, const Linspace<float>& ranges) {
        this->set_bearings(bearings);
        this->set_ranges(ranges);
    }
};


} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_SENSOR_INFO_H_