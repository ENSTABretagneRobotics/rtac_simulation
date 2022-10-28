#ifndef _DEF_RTAC_SIMULATION_HELPERS_OCULUS_ROSBAG_ITERATOR_H_
#define _DEF_RTAC_SIMULATION_HELPERS_OCULUS_ROSBAG_ITERATOR_H_

#include <iostream>
#include <utility>

#include <rosbag/bag.h>
#include <rosbag/view.h>
//#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <rtac_base/types/Pose.h>

#include <narval_oculus/Oculus.h>
#include <oculus_sonar/OculusPing.h>

namespace rtac { namespace simulation {

struct OculusDatum
{
    template <typename T>
    using BoostPtr  = boost::shared_ptr<T>;
    using OculusMsg = BoostPtr<oculus_sonar::OculusPing>;
    //using PoseMsg   = BoostPtr<nav_msgs::Odometry>;
    using PoseMsg   = BoostPtr<geometry_msgs::PoseStamped>;

    using Pose = rtac::types::Pose<float>;

    Pose      fixedPose_;
    OculusMsg pingMsg_;
    PoseMsg   poseMsg_;

    OculusDatum(const Pose& fixedPose = Pose()) :
        fixedPose_(fixedPose),
        pingMsg_(nullptr),
        poseMsg_(nullptr)
    {}

    void reset() { pingMsg_ = nullptr; poseMsg_ = nullptr; }
    bool is_complete() const { return pingMsg_ && poseMsg_; }

    const uint8_t* ping_data() const {
        if(!pingMsg_) {
            throw std::runtime_error("No ping data");
        }
        return reinterpret_cast<const uint8_t*>(pingMsg_->data.data());
    }

    const OculusSimplePingResult& ping_metadata() const {
        return *reinterpret_cast<const OculusSimplePingResult*>(this->ping_data());
    }

    Pose pose() const {
        if(!poseMsg_) {
            throw std::runtime_error("No pose data");
        }
        auto rosPose = poseMsg_->pose;
        Pose pose({(float)rosPose.position.x,
                   (float)rosPose.position.y,
                   (float)rosPose.position.z},
                  {(float)rosPose.orientation.w,
                   (float)rosPose.orientation.x,
                   (float)rosPose.orientation.y,
                   (float)rosPose.orientation.z});
        return pose * fixedPose_;
    }
};

/**
 * This class is a helper to easilly iterate over ping data and their
 * acquisition position.
 */
class OculusRosbagIterator
{
    public:

    using Pose = OculusDatum::Pose;

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
                         const std::string& poseTopic,
                         const Pose&   fixedPose = Pose());

    const std::string& bag_path()   const { return bagPath_;   }
    const std::string& ping_topic() const { return pingTopic_; }
    const std::string& pose_topic() const { return poseTopic_; }

    OculusDatum datum() const { return currentDatum_; }

    OculusDatum next(bool loop = true);
};

} //namespace simulation
} //namespace rtac


#endif //_DEF_RTAC_SIMULATION_HELPERS_OCULUS_ROSBAG_ITERATOR_H_
