#include <rtac_simulation/helpers/OculusRosbagIterator.h>

namespace rtac { namespace simulation {

OculusRosbagIterator::OculusRosbagIterator(const std::string& bagPath,
                                           const std::string& pingTopic,
                                           const std::string& poseTopic,
                                           const Pose& fixedPose) :
    bagPath_(bagPath),
    pingTopic_(pingTopic),
    poseTopic_(poseTopic),
    currentDatum_(fixedPose)
{
    std::cout << "Opening rosbag " << bagPath_ << " ... " << std::flush;
    rosbag_.open(bagPath_, rosbag::bagmode::Read);

    bagView_.addQuery(rosbag_, rosbag::TopicQuery({pingTopic_,poseTopic_}));
    bagIterator_ = bagView_.begin();

    std::cout << "Done." << std::endl;
}

OculusDatum OculusRosbagIterator::next(bool loop)
{
    auto it = bagIterator_;
    currentDatum_.reset();
    do {
        if(auto msg = it->instantiate<geometry_msgs::PoseStamped>()) {
            currentDatum_.poseMsg_ = msg;
        }
        if(auto msg = it->instantiate<oculus_sonar::OculusPing>()) {
            currentDatum_.pingMsg_ = msg;
        }

        it++;
        if(it == bagView_.end()) {
            if(!loop) {
                return OculusDatum();
            }
            it = bagView_.begin();
            currentDatum_.reset();
            continue;
        }
        if(currentDatum_.is_complete()) {
            bagIterator_ = it;
            return currentDatum_;
        }
    }
    while(it != bagIterator_);

    std::ostringstream oss;
    oss << "Infinite loop encountered while trying to iterate over topics ("
        << pingTopic_ << ", " << poseTopic_ << "in bag " << bagPath_ << ".";
    throw std::runtime_error(oss.str());
}

} //namespace simulation
} //namespace rtac




