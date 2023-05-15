#ifndef _DEF_RTAC_SIMULATION_POSE_SOURCE_H_
#define _DEF_RTAC_SIMULATION_POSE_SOURCE_H_

#include <memory>
#include <deque>
#include <iosfwd>

#include <rtac_base/types/Pose.h>

namespace rtac { namespace simulation {

class PoseSource
{
    public:

    using Ptr      = std::shared_ptr<PoseSource>;
    using ConstPtr = std::shared_ptr<const PoseSource>;
    using Pose = rtac::Pose<float>;

    protected:

    PoseSource() = default;

    public:

    // may return -1 when undefined
    virtual int  size()      const = 0;
    virtual int  remaining() const = 0;
    virtual Pose next_pose()       = 0;
};

class PoseSourceStatic : public PoseSource
{
    public:

    using Ptr      = std::shared_ptr<PoseSourceStatic>;
    using ConstPtr = std::shared_ptr<const PoseSourceStatic>;
    using Pose = rtac::Pose<float>;

    protected:

    Pose pose_;

    PoseSourceStatic(const Pose& pose = Pose()) : pose_(pose) {}

    public:

    static Ptr Create(const Pose& pose = Pose()) {
        return Ptr(new PoseSourceStatic(pose));
    }

    void set_pose(const Pose& pose) { pose_ = pose; }
    
    int  size()      const { return -1;    }
    int  remaining() const { return -1;    }
    Pose next_pose()       { return pose_; }
};

class PoseSourceList : public PoseSource
{
    public:

    using Ptr      = std::shared_ptr<PoseSourceList>;
    using ConstPtr = std::shared_ptr<const PoseSourceList>;
    using Pose     = rtac::Pose<float>;

    protected:

    std::deque<Pose> poses_;
    int              current_;

    PoseSourceList(const std::deque<Pose>& poses) :
        poses_(poses),
        current_(0)
    {}

    public:

    static Ptr Create(const std::deque<Pose>& poses) { return Ptr(new PoseSourceList(poses)); }
    static Ptr CreateFromCSV(const std::string& path,
                             unsigned int skipLines = 0,
                             char delimiter = ',');

    int  size()      const override { return poses_.size();            }
    int  remaining() const override { return poses_.size() - current_; }
    Pose next_pose()       override { return poses_[current_++];       }
};

}// namespace simulation
}// namespace rtac

#endif //_DEF_RTAC_SIMULATION_POSE_SOURCE_H_
