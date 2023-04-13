#ifndef _DEF_RTAC_SIMULATION_SENSOR_INSTANCE_1D_H_
#define _DEF_RTAC_SIMULATION_SENSOR_INSTANCE_1D_H_

#include <rtac_base/cuda/TextureVector.h>

#include <rtac_simulation/SensorInstance.h>

namespace rtac { namespace simulation {

class SensorInstance1D : public SensorInstance
{
    public:

    using Ptr      = std::shared_ptr<SensorInstance1D>;
    using ConstPtr = std::shared_ptr<const SensorInstance1D>;
    using Pose     = rtac::Pose<float>;

    using Sample = SimSample1D;
    using Bins   = rtac::cuda::CudaVector<VectorView<const Sample>>;

    protected:

    SensorInfo::ConstPtr info_;

    cuda::CudaVector<Sample> receivedSamples_;
    Bins bins_;

    cuda::Texture2D<float2> waveformData_;

    SensorInstance1D(const SensorInfo::ConstPtr& info,
                     const Pose& pose,
                     float soundCelerity);

    void generate_psf_data();
    void do_reduce(cuda::CudaVector<Complex<float>>& out,
                   const cuda::CudaVector<VectorView<const SimSample1D>>& bins) const;
    void* sample_pointer() { return receivedSamples_.data(); }

    public:

    unsigned int size()   const { return ranges_.size(); }

    const SensorInfo& info() const { return *info_; }

    KernelView1D<Complex<float>> kernel() const;

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
    void reduce_samples(cuda::CudaVector<T>& out)
    {
        sort(receivedSamples_);
        binner_.compute(bins_, receivedSamples_);
        this->do_reduce(out, bins_);
    }
};

class SensorInstance1D_Complex : public SensorInstance1D
{
    public:

    using Ptr      = std::shared_ptr<SensorInstance1D_Complex>;
    using ConstPtr = std::shared_ptr<const SensorInstance1D_Complex>;

    protected:

    cuda::CudaVector<Complex<float>> sensorOutput_;

    SensorInstance1D_Complex(const SensorInfo::ConstPtr& info,
                             const Pose& pose,
                             float soundCelerity) :
        SensorInstance1D(info, pose, soundCelerity)
    {
        this->set_ranges(info->ranges());
    }

    public:

    static Ptr Create(const SensorInfo::ConstPtr& info,
                      const Pose& pose = Pose(),
                      float soundCelerity = 1500.0)
    {
        return Ptr(new SensorInstance1D_Complex(info, pose, soundCelerity));
    }

    bool is_complex() const { return true; }

    const cuda::CudaVector<Complex<float>>& output() const { return sensorOutput_; }

    void compute_output()
    {
        sensorOutput_.resize(this->size());
        this->reduce_samples(sensorOutput_);
    }
};


} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_SENSOR_INSTANCE_1D_H_
