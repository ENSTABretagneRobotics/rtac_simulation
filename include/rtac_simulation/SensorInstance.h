#ifndef _DEF_RTAC_SIMULATION_SENSOR_INSTANCE_H_
#define _DEF_RTAC_SIMULATION_SENSOR_INSTANCE_H_

#include <memory>

#include <rtac_base/types/Linspace.h>
#include <rtac_base/containers/Image.h>
#include <rtac_base/cuda/Texture2D.h>

#include <rtac_simulation/SensorInfo.h>
#include <rtac_simulation/Waveform.h>
#include <rtac_simulation/Binner.h>
#include <rtac_simulation/ReductionKernel.h>

namespace rtac { namespace simulation {

class SensorInstance
{
    public:

    using Ptr      = std::shared_ptr<SensorInstance>;
    using ConstPtr = std::shared_ptr<const SensorInstance>;

    protected:

    SensorInfo2D_2::ConstPtr info_;

    Linspace<float> ranges_;
    Waveform::Ptr   waveform_;
    float           soundCelerity_;

    Binner binner_;
    cuda::Texture2D<float2> psfData_;

    SensorInstance(const SensorInfo2D_2::ConstPtr& info,
                   float soundCelerity);

    void generate_psf_data();
    void do_reduce(Image<Complex<float>, cuda::DeviceVector>& out,
                   const cuda::DeviceVector<VectorView<const SimSample2D>>& bins) const;

    public:

    static Ptr Create(const SensorInfo2D_2::ConstPtr& info,
                      float soundCelerity = 1500.0)
    {
        return Ptr(new SensorInstance(info, soundCelerity));
    }

    unsigned int width()  const { return info_->bearings().size(); }
    unsigned int height() const { return ranges_.size(); }
    unsigned int size()   const { return this->width() * this->height(); }

    //const SensorInfo2D_2::ConstPtr& info() const { return info_; }
    const Linspace<float>& ranges() const { return ranges_; }
    cuda::TextureVectorView<float> bearings_view() const {
        return info_->bearings_view();
    }

    BeamDirectivity::ConstPtr beam_directivity() const { return info_->beam_directivity(); }
    Waveform::ConstPtr waveform() const { return waveform_; }
    float sound_celerity() const { return soundCelerity_; }
    Directivity::ConstPtr directivity() const { return info_->directivity(); }

    void set_ranges(const Linspace<float>& ranges, float soundCelerity);
    void set_ranges(const Linspace<float>& ranges);
    void set_ranges(float maxRange, unsigned int rangeCount);
    KernelView2D<Complex<float>> kernel() const;

    template <typename T>
    void reduce_samples(Image<Complex<float>, cuda::DeviceVector>& out,
                        const cuda::DeviceVector<T>& samples)
    {
        cuda::DeviceVector<VectorView<const T>> bins;
        binner_.compute(bins, samples);
        this->do_reduce(out, bins);
        //sparse_convolve_2d(out, *this, bins);
    }
};

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_SENSOR_INSTANCE_H(*ths_
