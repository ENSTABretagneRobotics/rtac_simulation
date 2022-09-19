#ifndef _DEF_RTAC_SIMULATION_POLAR_TARGET_H_
#define _DEF_RTAC_SIMULATION_POLAR_TARGET_H_

#include <iostream>
#include <memory>

#include <rtac_base/types/VectorView.h>
#include <rtac_base/types/Image.h>

#include <rtac_simulation/common.h>
#include <rtac_simulation/PolarKernel2D.h>

namespace rtac { namespace simulation {

template <typename T>
struct PolarTargetView2D : public ImageView<T>
{
    public:

    using value_type = T;
    using Shape = typename ImageView<T>::Shape;

    protected:

    const float* bearings_;
    const float* ranges_;

    public:

    PolarTargetView2D(DeviceImage<T>& image,
                      const DeviceVector<float>& bearings,
                      const DeviceVector<float>& ranges) :
        ImageView<T>(image.view()),
        bearings_(bearings.data()),
        ranges_(ranges.data())
    {}

    #ifdef RTAC_CUDACC
    __device__ float bearing(uint32_t w) const { return bearings_[w]; }
    __device__ float range(uint32_t h)   const { return ranges_[h];   }
    #endif //RTAC_KERNEL
};

template <typename T>
class PolarTarget2D
{
    public:

    using value_type = T;

    using Ptr      = std::shared_ptr<PolarTarget2D<T>>;
    using ConstPtr = std::shared_ptr<const PolarTarget2D<T>>;

    protected:
    
    DeviceImage<T>      data_;
    DeviceVector<float> bearings_;
    DeviceVector<float> ranges_;
    float rangeMin_;
    float rangeMax_;

    PolarTarget2D(const std::vector<float>& bearings,
                  const std::vector<float>& ranges)
    {
        this->update(bearings, ranges);
    }

    public:

    static Ptr Create(const std::vector<float>& bearings,
                      const std::vector<float>& ranges)
    {
        return Ptr(new PolarTarget2D<T>(bearings, ranges));
    }

    const DeviceImage<T>& data() const { return data_; }

    std::size_t range_count()   const { return ranges_.size(); }
    std::size_t bearing_count() const { return bearings_.size(); }

    float range_min()         const { return rangeMin_; }
    float range_max()         const { return rangeMax_; }
    float range_resolution()  const { 
        return (rangeMax_ - rangeMin_) / (ranges_.size() - 1);
    }

    void update(const std::vector<float>& bearings,
                const std::vector<float>& ranges)
    {
        data_.resize({(unsigned int)bearings.size(),
                      (unsigned int)ranges.size()});
        bearings_ = bearings;
        ranges_   = ranges;
        rangeMin_ = ranges.front();
        rangeMax_ = ranges.back();
    }

    PolarTargetView2D<T> view() {
        return PolarTargetView2D(data_, bearings_, ranges_);
    }
};

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_POLAR_TARGET_H_
