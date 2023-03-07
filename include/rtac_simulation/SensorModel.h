#ifndef _DEF_RTAC_SIMULATION_SENSOR_MODEL_H_
#define _DEF_RTAC_SIMULATION_SENSOR_MODEL_H_

#include <rtac_base/types/Linspace.h>
#include <rtac_base/containers/ScaledImage.h>
#include <rtac_base/cuda/DeviceVector.h>
#include <rtac_base/cuda/TextureDim.h>

#include <rtac_simulation/ReductionKernel.h>
#include <rtac_simulation/Binner.h>
#include <rtac_simulation/PointSpreadFunction.h>
#include <rtac_simulation/reductions_2.h>

#include <rtac_simulation/SensorInfo.h>

namespace rtac { namespace simulation {

template <typename T, typename KT = T>
class SensorModel2D
{
    public:

    using Ptr      = std::shared_ptr<SensorModel2D<T,KT>>;
    using ConstPtr = std::shared_ptr<const SensorModel2D<T,KT>>;

    using RangeDimension   = rtac::LinearDim;
    using BearingDimension = rtac::cuda::TextureDim;

    using DataImage = ScaledImage<T, BearingDimension, RangeDimension, cuda::DeviceVector>;
    using DataView  = ScaledImageView<T, rtac::cuda::TextureDimView, rtac::LinearDim>;

    protected:

    DataImage    data_;
    Kernel2D<KT> pointSpreadFunction_;
    Binner       binner_;

    SensorModel2D(const std::vector<float>& bearings,
                  const LinearDim&          ranges,
                  const Kernel2D<KT>&       psf) :
        data_(rtac::cuda::TextureDim(bearings), ranges),
        pointSpreadFunction_(psf),
        binner_(ranges.size(), ranges.bounds(), psf.y_span())
    {}

    public:

    static Ptr Create(const std::vector<float>& bearings,
                      const LinearDim&          ranges,
                      const Kernel2D<KT>&       psf)
    {
        return Ptr(new SensorModel2D<T,KT>(bearings, ranges, psf));
    }

    unsigned int width()  const { return data_.width();  }
    unsigned int height() const { return data_.height(); }

    void reconfigure(const std::vector<float>& bearings, const LinearDim ranges) {
        data_.reconfigure(rtac::cuda::TextureDim(bearings), ranges);
        binner_ = Binner(ranges.size(), ranges.bounds());
    }
    void set_bearings(const std::vector<float>& bearings) {
        data_.set_width_dim(rtac::cuda::TextureDim(bearings));
    }
    void set_ranges(const LinearDim& ranges) {
        data_.set_height_dim(ranges);
        binner_ = Binner(ranges.size(), ranges.bounds());
    }
    void set_ranges(unsigned int count, const Bounds<float>& bounds) {
        this->set_ranges(LinearDim(count, bounds));
    }
    
    const DataImage& data() const { return data_; }
          DataImage& data()       { return data_; }

    const Kernel2D<KT>& point_spread_function() const { return pointSpreadFunction_; }
          Kernel2D<KT>& point_spread_function()       { return pointSpreadFunction_; }
    
    template <typename T2>
    void reduce_samples(const rtac::cuda::DeviceVector<T2>& samples)
    {
        rtac::cuda::DeviceVector<rtac::VectorView<const T2>> bins;
        binner_.compute(bins, samples);
        sparse_convolve_2d(*this, bins);
    }
};

template <typename T>
class SensorModel2D_2
{
    public:

    using Ptr      = std::shared_ptr<SensorModel2D_2<T>>;
    using ConstPtr = std::shared_ptr<const SensorModel2D_2<T>>;

    using RangeDimension   = rtac::LinearDim;
    using BearingDimension = rtac::cuda::TextureDim;

    using DataImage = ScaledImage<T, BearingDimension, RangeDimension, cuda::DeviceVector>;
    using DataView  = ScaledImageView<T, rtac::cuda::TextureDimView, rtac::LinearDim>;

    protected:

    DataImage data_;
    Binner    binner_;
    PointSpreadFunction2D::Ptr psf_;

    SensorModel2D_2(const std::vector<float>& bearings,
                    const LinearDim&          ranges,
                    const PointSpreadFunction2D::Ptr& psf) :
        data_(rtac::cuda::TextureDim(bearings), ranges),
        binner_(ranges.size(), ranges.bounds(), psf->range_span()),
        psf_(psf)
    {}

    public:

    static Ptr Create(const std::vector<float>& bearings,
                      const LinearDim&          ranges,
                      const PointSpreadFunction2D::Ptr& psf)
    {
        return Ptr(new SensorModel2D_2<T>(bearings, ranges, psf));
    }

    unsigned int width()  const { return data_.width();  }
    unsigned int height() const { return data_.height(); }

    void reconfigure(const std::vector<float>& bearings, const LinearDim ranges) {
        data_.reconfigure(rtac::cuda::TextureDim(bearings), ranges);
        binner_ = Binner(ranges.size(), ranges.bounds());
        psf_->set_pulse_length(2*ranges.bounds().length() / (ranges.size() - 1));
    }
    void set_bearings(const std::vector<float>& bearings) {
        data_.set_width_dim(rtac::cuda::TextureDim(bearings));
    }
    void set_ranges(const LinearDim& ranges) {
        data_.set_height_dim(ranges);
        binner_ = Binner(ranges.size(), ranges.bounds());
        psf_->set_pulse_length(2*ranges.bounds().length() / (ranges.size() - 1));
    }
    void set_ranges(unsigned int count, const Bounds<float>& bounds) {
        this->set_ranges(LinearDim(count, bounds));
    }
    
    const DataImage& data() const { return data_; }
          DataImage& data()       { return data_; }

    PointSpreadFunction2D::ConstPtr point_spread_function() const { return psf_; }
    PointSpreadFunction2D::Ptr      point_spread_function()       { return psf_; }
    
    template <typename T2>
    void reduce_samples(const rtac::cuda::DeviceVector<T2>& samples)
    {
        rtac::cuda::DeviceVector<rtac::VectorView<const T2>> bins;
        binner_.compute(bins, samples);
        sparse_convolve_2d(*this, bins);
    }
};

class SensorModel2D_Base
{
    public:

    using Ptr      = std::shared_ptr<SensorModel2D_Base>;
    using ConstPtr = std::shared_ptr<const SensorModel2D_Base>;

    protected:

    SensorInfo2D::Ptr info_;
    Binner            binner_;

    cuda::TextureVector<float> bearingsData_;

    SensorModel2D_Base(const SensorInfo2D::Ptr& info) :
        info_(info),
        bearingsData_(info_->bearings())
    {}

    public:

    SensorInfo2D::ConstPtr info() const { return info_; }

    unsigned int width()  const { return info_->width();  }
    unsigned int height() const { return info_->height(); }
    unsigned int size()   const { return this->width()*this->height(); }

    const std::vector<float>& bearings() const { return info_->bearings(); }
    const Linspace<float>&    ranges()   const { return info_->ranges();   }
    PointSpreadFunction2D::ConstPtr point_spread_function() const {
        return info_->point_spread_function();
    }
    Directivity::ConstPtr directivity() const { 
        return info_->directivity();
    }

    cuda::TextureVectorView<float> bearings_view() const {
        return bearingsData_.view();
    }

    template <typename T2>
    void reduce_samples(const rtac::cuda::DeviceVector<T2>& samples)
    {
        rtac::cuda::DeviceVector<rtac::VectorView<const T2>> bins;
        binner_.compute(bins, samples);
        sparse_convolve_2d(*this, bins);
    }

    virtual void reconfigure(const std::vector<float>& bearings,
                             const Linspace<float>& ranges) = 0;
    virtual void set_bearings(const std::vector<float>& bearings) = 0;
    virtual void set_ranges(const Linspace<float>& ranges) = 0;
    virtual bool is_complex() const = 0;
};

template <typename T>
class SensorModel2D_3 : public SensorModel2D_Base
{
    public:

    using Ptr      = std::shared_ptr<SensorModel2D_3>;
    using ConstPtr = std::shared_ptr<const SensorModel2D_3>;

    protected:

    cuda::DeviceVector<T> data_;

    SensorModel2D_3(const SensorInfo2D::Ptr& info) : 
        SensorModel2D_Base(info),
        data_(info->size())
    {}

    public:

    const cuda::DeviceVector<T>& data() const { return data_; }

    ImageView<T> data_view() { 
        return ImageView<T>(this->width(), this->height(), data_.data());
    }
    ImageView<const T> data_view() const { 
        return ImageView<const T>(this->width(), this->height(), data_.data());
    }

    void reconfigure(const std::vector<float>& bearings, const Linspace<float>& ranges)
    {
        this->info_->reconfigure(bearings, ranges);
        this->binner_.reconfigure(info_->ranges(), ranges.resolution());
        data_.resize(this->size());
    }
    void set_bearings(const std::vector<float>& bearings) {
        this->info_->set_bearings(bearings);
        data_.resize(this->size());
    }
    void set_ranges(const Linspace<float>& ranges) {
        this->info_->set_ranges(ranges);
        this->binner_.reconfigure(info_->ranges(), ranges.resolution());
        data_.resize(this->size());
    }
    void set_ranges(float maxRange, unsigned int count) {
        this->set_ranges(Linspace<float>({info_->ranges().lower(), maxRange}, count));
    }
};

class SensorModel2D_Real : public SensorModel2D_3<float>
{
    public:

    using Ptr      = std::shared_ptr<SensorModel2D_Real>;
    using ConstPtr = std::shared_ptr<const SensorModel2D_Real>;

    protected:

    SensorModel2D_Real(const SensorInfo2D::Ptr& info) : 
        SensorModel2D_3<float>(info)
    {}

    public:

    static Ptr Create(const SensorInfo2D::Ptr& info) {
        return Ptr(new SensorModel2D_Real(info));
    }
    bool is_complex() const { return false; }
};

class SensorModel2D_Complex : public SensorModel2D_3<Complex<float>>
{
    public:

    using Ptr      = std::shared_ptr<SensorModel2D_Complex>;
    using ConstPtr = std::shared_ptr<const SensorModel2D_Complex>;

    protected:

    SensorModel2D_Complex(const SensorInfo2D::Ptr& info) : 
        SensorModel2D_3<Complex<float>>(info)
    {}

    public:

    static Ptr Create(const SensorInfo2D::Ptr& info) {
        return Ptr(new SensorModel2D_Complex(info));
    }
    bool is_complex() const { return true; }
};

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_SENSOR_MODEL_H_
