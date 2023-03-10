#ifndef _DEF_RTAC_SIMULATION_SENSOR_INSTANCE_H_
#define _DEF_RTAC_SIMULATION_SENSOR_INSTANCE_H_

#include <memory>

#include <rtac_base/types/Linspace.h>
#include <rtac_base/types/Pose.h>
#include <rtac_base/containers/Image.h>
#include <rtac_base/cuda/Texture2D.h>

#include <rtac_simulation/SensorInfo.h>
#include <rtac_simulation/Waveform.h>
#include <rtac_simulation/Binner.h>
#include <rtac_simulation/ReductionKernel.h>

#include <rtac_simulation/Receiver.h>

namespace rtac { namespace simulation {

class SensorInstance2D
{
    public:

    using Ptr      = std::shared_ptr<SensorInstance2D>;
    using ConstPtr = std::shared_ptr<const SensorInstance2D>;
    using Pose     = rtac::Pose<float>;

    using Sample = SimSample2D;
    using Bins   = rtac::cuda::DeviceVector<VectorView<const Sample>>;

    protected:

    SensorInfo2D_2::ConstPtr info_;

    Linspace<float> ranges_;
    Waveform::Ptr   waveform_;
    Pose            pose_;
    float           soundCelerity_;

    cuda::DeviceVector<Sample> receivedSamples_;
    Bins bins_;

    Binner binner_;
    cuda::Texture2D<float2> psfData_;

    SensorInstance2D(const SensorInfo2D_2::ConstPtr& info,
                     const Pose& pose,
                     float soundCelerity);

    void generate_psf_data();
    void do_reduce(Image<Complex<float>, cuda::DeviceVector>& out,
                   const cuda::DeviceVector<VectorView<const SimSample2D>>& bins) const;

    public:

    unsigned int width()  const { return info_->bearings().size(); }
    unsigned int height() const { return ranges_.size(); }
    unsigned int size()   const { return this->width() * this->height(); }

    //const SensorInfo2D_2::ConstPtr& info() const { return info_; }
    const Linspace<float>& ranges() const { return ranges_; }
    const std::vector<float>& bearings() const { return info_->bearings(); }
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

    void set_sample_count(unsigned int count) { receivedSamples_.resize(count); }
    const cuda::DeviceVector<Sample>& samples() const { return receivedSamples_; }
          cuda::DeviceVector<Sample>& samples()       { return receivedSamples_; }
    const Pose& pose() const { return pose_; }
          Pose& pose()       { return pose_; }

    ReceiverView2<Sample> receiver_view()
    {
        ReceiverView2<Sample> res;
        res.pose        = pose_;
        res.directivity = this->directivity()->view();
        res.size        = receivedSamples_.size(); // replace this with VectorView ?
        res.samples     = receivedSamples_.data();

        return res;
    }

    template <typename T>
    void reduce_samples(Image<T, cuda::DeviceVector>& out)
    {
        sort(receivedSamples_);
        binner_.compute(bins_, receivedSamples_);
        this->do_reduce(out, bins_);
    }

    virtual bool is_complex() const = 0;
    virtual void compute_output() = 0;
};

class SensorInstance2D_Complex : public SensorInstance2D
{
    public:

    using Ptr      = std::shared_ptr<SensorInstance2D_Complex>;
    using ConstPtr = std::shared_ptr<const SensorInstance2D_Complex>;

    protected:

    Image<Complex<float>, cuda::DeviceVector> sensorOutput_;

    SensorInstance2D_Complex(const SensorInfo2D_2::ConstPtr& info,
                             const Pose& pose,
                             float soundCelerity) :
        SensorInstance2D(info, pose, soundCelerity)
    {}

    public:

    static Ptr Create(const SensorInfo2D_2::ConstPtr& info,
                      const Pose& pose,
                      float soundCelerity = 1500.0)
    {
        return Ptr(new SensorInstance2D_Complex(info, pose, soundCelerity));
    }

    bool is_complex() const { return true; }

    const Image<Complex<float>, cuda::DeviceVector>& output() const { return sensorOutput_; }

    void compute_output()
    {
        sensorOutput_.resize({this->width(), this->height()});
        this->reduce_samples(sensorOutput_);
    }
};


} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_SENSOR_INSTANCE_H(*ths_
