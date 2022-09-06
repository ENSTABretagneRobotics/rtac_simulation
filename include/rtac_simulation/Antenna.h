#ifndef _DEF_RTAC_SIMULATION_ANTENNA_H_
#define _DEF_RTAC_SIMULATION_ANTENNA_H_

#include <iostream>
#include <memory>

#include <rtac_base/types/Pose.h>

#include <rtac_simulation/geometry.h>
#include <rtac_simulation/Directivity.h>

namespace rtac { namespace simulation {

template <typename T>
struct AntennaView {
    DevicePose<T>      pose;
    DirectivityView<T> directivity;
};

template <typename T>
class Antenna
{
    public:
   
    using Ptr      = std::shared_ptr<Antenna<T>>;
    using ConstPtr = std::shared_ptr<const Antenna<T>>;

    using Pose = rtac::types::Pose<T>;

    protected:

    Pose                 pose_;
    Directivity<T>::Ptr directivity_;

    Antenna(Directivity<T>::Ptr directivity, const Pose& pose = Pose()) :
        pose_(pose), directivity_(directivity)
    {}

    const Pose& pose() const { return pose_; }
    Pose&       pose()       { return pose_; }

    Directivity<T>::ConstPtr directivity() const { return directivity_; }
    Directivity<T>::Ptr      directivity()       { return directivity_; }

    AntennaView<T> view() const {
        AntennaView<T> res;
        res.pose        = pose_;
        res.directivity = directivity_.view();
        return res;
    }
};

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_ANTENNA_H_
