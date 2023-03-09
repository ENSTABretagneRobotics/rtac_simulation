#include <rtac_simulation/SensorInstance.h>

#include <rtac_base/containers/Image.h>

namespace rtac { namespace simulation {

SensorInstance::SensorInstance(const SensorInfo2D_2::ConstPtr& info,
                               float soundCelerity) :
    info_(info),
    ranges_(info_->ranges()),
    waveform_(info_->waveform()->copy()),
    soundCelerity_(soundCelerity)
{}


void SensorInstance::generate_psf_data()
{
    const auto& waveform = *waveform_;
    const auto& beam     = *info_->beam_directivity();
    Image<Complex<float>> data(beam.size(), waveform.size());

    for(unsigned int h = 0; h < data.height(); h++) {
        for(unsigned int w = 0; w < data.width(); w++) {
            data(h,w) = waveform[h] * beam[w];
        }
    }

    psfData_.set_image(data.width(), data.height(), (const float2*)data.data());
}

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

KernelView2D<Complex<float>> SensorInstance::kernel() const
{
    KernelView2D<Complex<float>> kernel;
    kernel.xScaling_ = float2{1.0f / this->beam_directivity()->span(), 0.5f};
    kernel.yScaling_ = float2{1.0f / waveform_->duration()*soundCelerity_,   0.5f};
    kernel.function_ = psfData_.texture();
    return kernel;
}

} //namespace simulation
} //namespace rtac
