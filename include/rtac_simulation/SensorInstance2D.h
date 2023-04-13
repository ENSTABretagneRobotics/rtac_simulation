#ifndef _DEF_RTAC_SIMULATION_SENSOR_INSTANCE_2D_H_
#define _DEF_RTAC_SIMULATION_SENSOR_INSTANCE_2D_H_

#include <rtac_simulation/SensorInstance.h>

namespace rtac { namespace simulation {

class SensorInstance2D : public SensorInstance
{
    public:

    using Ptr      = std::shared_ptr<SensorInstance2D>;
    using ConstPtr = std::shared_ptr<const SensorInstance2D>;
    using Pose     = rtac::Pose<float>;

    using Sample = SimSample2D;
    using Bins   = rtac::cuda::CudaVector<VectorView<const Sample>>;

    protected:

    SensorInfo2D::ConstPtr info_;

    cuda::CudaVector<Sample> receivedSamples_;
    Bins bins_;

    cuda::Texture2D<float2> psfData_;

    SensorInstance2D(const SensorInfo2D::ConstPtr& info,
                     const Pose& pose,
                     float soundCelerity);

    void generate_psf_data();
    void do_reduce(Image<Complex<float>, cuda::CudaVector>& out,
                   const cuda::CudaVector<VectorView<const SimSample2D>>& bins) const;
    
    void* sample_pointer() { return receivedSamples_.data(); }

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
    unsigned int sample_count() const { return receivedSamples_.size(); }
    const cuda::CudaVector<Sample>& samples() const { return receivedSamples_; }
          cuda::CudaVector<Sample>& samples()       { return receivedSamples_; }

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
    void reduce_samples(Image<T, cuda::CudaVector>& out)
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

    Image<Complex<float>, cuda::CudaVector> sensorOutput_;

    SensorInstance2D_Complex(const SensorInfo2D::ConstPtr& info,
                             const Pose& pose,
                             float soundCelerity) :
        SensorInstance2D(info, pose, soundCelerity)
    {
        this->set_ranges(info->ranges());
    }

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

    const Image<Complex<float>, cuda::CudaVector>& output() const { return sensorOutput_; }

    void compute_output()
    {
        sensorOutput_.resize({this->width(), this->height()});
        this->reduce_samples(sensorOutput_);
    }
};


} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_SENSOR_INSTANCE_2D_H_
