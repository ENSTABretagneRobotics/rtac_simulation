#ifndef _DEF_RTAC_SIMULATION_ANTENNA_H_
#define _DEF_RTAC_SIMULATION_ANTENNA_H_

#include <iostream>
#include <memory>

#include <rtac_simulation/common.h>
#include <rtac_simulation/geometry.h>
#include <rtac_simulation/Directivity.h>

namespace rtac { namespace simulation {

struct AntennaView {
    DevicePose<float> pose;
    DirectivityView   directivity;
};

class Antenna
{
    public:
   
    using Ptr      = std::shared_ptr<Antenna>;
    using ConstPtr = std::shared_ptr<const Antenna>;

    using DataShape = Directivity::DataShape;

    protected:

    Pose  pose_;
    typename Directivity::ConstPtr directivity_;

    public:

    Antenna(typename Directivity::ConstPtr directivity, const Pose& pose = Pose()) :
        pose_(pose), directivity_(directivity)
    {}

    const Pose& pose() const { return pose_; }
    Pose&       pose()       { return pose_; }

    typename Directivity::ConstPtr directivity() const { return directivity_; }

    AntennaView view() const {
        AntennaView res;
        res.pose        = pose_;
        res.directivity = directivity_->view();
        return res;
    }
};

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_ANTENNA_H_
