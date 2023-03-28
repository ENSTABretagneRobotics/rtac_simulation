#include <rtac_simulation/SensorInstance.h>

#include <rtac_base/containers/Image.h>
#include <rtac_base/cuda/reductions.hcu>

namespace rtac { namespace simulation {

SensorInstance::SensorInstance(const SensorInfo::ConstPtr& info,
                               const Pose& pose,
                               float soundCelerity) :
    ranges_(info->ranges()),
    waveform_(info->waveform()->copy()),
    pose_(pose),
    soundCelerity_(soundCelerity)
{}

void SensorInstance::set_ranges(const Linspace<float>& ranges, float soundCelerity)
{
    soundCelerity_ = soundCelerity;
    this->set_ranges(ranges);
}

void SensorInstance::set_ranges(const Linspace<float>& ranges)
{
    ranges_ = ranges;
    binner_.reconfigure(ranges, ranges.resolution());
    waveform_->set_duration(2*ranges.resolution() / soundCelerity_);
    this->generate_psf_data();
}

void SensorInstance::set_ranges(float maxRange, unsigned int rangeCount)
{
    this->set_ranges(Linspace<float>(ranges_.lower(), maxRange, rangeCount));
}


} //namespace simulation
} //namespace rtac
