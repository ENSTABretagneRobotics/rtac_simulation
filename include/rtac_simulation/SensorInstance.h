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

class SensorInstance : public std::enable_shared_from_this<SensorInstance>
{
    public:

    using Ptr      = std::shared_ptr<SensorInstance>;
    using ConstPtr = std::shared_ptr<const SensorInstance>;
    using Pose     = rtac::Pose<float>;

    protected:

    Linspace<float> ranges_;
    Waveform::Ptr   waveform_;
    Pose            pose_;
    float           soundCelerity_;
    Binner          binner_;

    SensorInstance(const SensorInfo::ConstPtr& info,
                   const Pose& pose,
                   float soundCelerity);

    virtual void generate_psf_data() = 0;

    public:

    Ptr      ptr()       { return this->shared_from_this(); }
    ConstPtr ptr() const { return this->shared_from_this(); }

    const Pose& pose() const { return pose_; }
          Pose& pose()       { return pose_; }
    const Linspace<float>& ranges() const { return ranges_; }
    Waveform::ConstPtr waveform() const { return waveform_; }
    float sound_celerity() const { return soundCelerity_; }
    Directivity::ConstPtr directivity() const { return this->info().directivity(); }

    void set_ranges(const Linspace<float>& ranges, float soundCelerity);
    void set_ranges(const Linspace<float>& ranges);
    void set_ranges(float maxRange, unsigned int rangeCount);

    virtual const SensorInfo& info() const = 0;
    virtual void set_sample_count(unsigned int count) = 0;
    virtual bool is_complex() const = 0;
    virtual void compute_output() = 0;
};


class SensorInstance2D : public SensorInstance
{
    public:

    using Ptr      = std::shared_ptr<SensorInstance2D>;
    using ConstPtr = std::shared_ptr<const SensorInstance2D>;
    using Pose     = rtac::Pose<float>;

    using Sample = SimSample2D;
    using Bins   = rtac::cuda::DeviceVector<VectorView<const Sample>>;

    protected:

    SensorInfo2D::ConstPtr info_;

    cuda::DeviceVector<Sample> receivedSamples_;
    Bins bins_;

    cuda::Texture2D<float2> psfData_;

    SensorInstance2D(const SensorInfo2D::ConstPtr& info,
                     const Pose& pose,
                     float soundCelerity);

    void generate_psf_data();
    void do_reduce(Image<Complex<float>, cuda::DeviceVector>& out,
                   const cuda::DeviceVector<VectorView<const SimSample2D>>& bins) const;

    public:

    unsigned int width()  const { return info_->bearings().size(); }
    unsigned int height() const { return ranges_.size(); }
    unsigned int size()   const { return this->width() * this->height(); }

    const SensorInfo2D& info() const { return *info_; }
    const std::vector<float>& bearings() const { return info_->bearings(); }
    cuda::TextureVectorView<float> bearings_view() const {
        return info_->bearings_view();
    }

    BeamDirectivity::ConstPtr beam_directivity() const { return info_->beam_directivity(); }

    KernelView2D<Complex<float>> kernel() const;

    void set_sample_count(unsigned int count) { receivedSamples_.resize(count); }
    const cuda::DeviceVector<Sample>& samples() const { return receivedSamples_; }
          cuda::DeviceVector<Sample>& samples()       { return receivedSamples_; }

    ReceiverView<Sample> receiver_view()
    {
        ReceiverView<Sample> res;
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
};

class SensorInstance2D_Complex : public SensorInstance2D
{
    public:

    using Ptr      = std::shared_ptr<SensorInstance2D_Complex>;
    using ConstPtr = std::shared_ptr<const SensorInstance2D_Complex>;

    protected:

    Image<Complex<float>, cuda::DeviceVector> sensorOutput_;

    SensorInstance2D_Complex(const SensorInfo2D::ConstPtr& info,
                             const Pose& pose,
                             float soundCelerity) :
        SensorInstance2D(info, pose, soundCelerity)
    {}

    public:

    static Ptr Create(const SensorInfo2D::ConstPtr& info,
                      const Pose& pose = Pose(),
                      float soundCelerity = 1500.0)
    {
        return Ptr(new SensorInstance2D_Complex(info, pose, soundCelerity));
    }

    static Ptr Create(const SensorInfo::ConstPtr& info,
                      const Pose& pose = Pose(),
                      float soundCelerity = 1500.0)
    {
        auto info2d = std::dynamic_pointer_cast<const SensorInfo2D>(info);
        if(!info2d) {
            throw std::runtime_error(
                "A SensorInstance2D must be created with a SensorInfo2D instance");
        }
        return Ptr(new SensorInstance2D_Complex(info2d, pose, soundCelerity));
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
