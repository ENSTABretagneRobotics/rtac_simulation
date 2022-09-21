#ifndef _DEF_RTAC_SIMULATION_HELPERS_OCULUS_ROSBAG_ITERATOR_H_
#define _DEF_RTAC_SIMULATION_HELPERS_OCULUS_ROSBAG_ITERATOR_H_

#include <iostream>
#include <utility>

#include <rosbag/bag.h>
#include <rosbag/view.h>
//#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <narval_oculus/Oculus.h>
#include <oculus_sonar/OculusPing.h>

namespace rtac { namespace simulation {

/**
 * This class is a helper to easilly iterate over ping data and their
 * acquisition position.
 */
class OculusRosbagIterator
{
    public:
    
    template <typename T>
    using BoostPtr    = boost::shared_ptr<T>;
    using OculusMsg   = BoostPtr<oculus_sonar::OculusPing>;
    //using PoseMsg     = BoostPtr<nav_msgs::Odometry>;
    using PoseMsg     = BoostPtr<geometry_msgs::PoseStamped>;
    using OculusDatum = std::pair<OculusMsg, PoseMsg>;

    protected:

    std::string bagPath_;
    std::string pingTopic_;
    std::string poseTopic_;

    rosbag::Bag            rosbag_;
    rosbag::View           bagView_;
    rosbag::View::iterator bagIterator_;

    OculusDatum currentDatum_;

    public:

    OculusRosbagIterator(const std::string& bagPath,
                         const std::string& pingTopic,
                         const std::string& poseTopic);

    const std::string& bag_path()   const { return bagPath_;   }
    const std::string& ping_topic() const { return pingTopic_; }
    const std::string& pose_topic() const { return poseTopic_; }

    OculusDatum datum() const { return currentDatum_; }

    OculusDatum next();
};

} //namespace simulation
} //namespace rtac


#endif //_DEF_RTAC_SIMULATION_HELPERS_OCULUS_ROSBAG_ITERATOR_H_
