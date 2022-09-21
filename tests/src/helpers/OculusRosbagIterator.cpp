#include <rtac_simulation/helpers/OculusRosbagIterator.h>

namespace rtac { namespace simulation {

OculusRosbagIterator::OculusRosbagIterator(const std::string& bagPath,
                                           const std::string& pingTopic,
                                           const std::string& poseTopic) :
    bagPath_(bagPath),
    pingTopic_(pingTopic),
    poseTopic_(poseTopic),
    currentDatum_({nullptr,nullptr})
{
    std::cout << "Opening rosbag " << bagPath_ << " ... " << std::flush;
    rosbag_.open(bagPath_, rosbag::bagmode::Read);

    bagView_.addQuery(rosbag_, rosbag::TopicQuery({pingTopic_,poseTopic_}));
    bagIterator_ = bagView_.begin();

    std::cout << "Done." << std::endl;
}

OculusRosbagIterator::OculusDatum OculusRosbagIterator::next()
{
    auto it = bagIterator_;
    do {
        if(auto msg = it->instantiate<geometry_msgs::PoseStamped>()) {
            currentDatum_.second = msg;
        }
        if(auto msg = it->instantiate<oculus_sonar::OculusPing>()) {
            currentDatum_.first = msg;
        }

        it++;
        if(it == bagView_.end()) {
            it = bagView_.begin();
            currentDatum_.first  = nullptr;
            currentDatum_.second = nullptr;
            continue;
        }
        if(currentDatum_.first && currentDatum_.second) {
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




