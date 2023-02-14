#include <rtac_simulation/Binner.h>

#include <thrust/execution_policy.h>
#include <thrust/device_ptr.h>
#include <thrust/transform.h>

namespace rtac { namespace simulation {

Binner::Binner(unsigned int binCount, const Bounds<float>& bounds,
               float margin)
{
    this->reconfigure(binCount, bounds, margin);
}

void Binner::reconfigure(unsigned int binCount, const Bounds<float>& bounds,
                         float margin)
{
    binCount_ = binCount;
    bounds_   = bounds;
    margin_   = margin;
}

void Binner::compute_keys(const rtac::cuda::DeviceVector<SimSample1D>& samples)
{
    keys_.resize(samples.size());
    thrust::transform(thrust::device_pointer_cast(samples.begin()),
                      thrust::device_pointer_cast(samples.end()),
                      thrust::device_pointer_cast(keys_.begin()),
                      Key(this->bin_count(), bounds_.lower, bounds_.upper, margin_));
}

void Binner::compute_keys(const rtac::cuda::DeviceVector<SimSample2D>& samples)
{
    keys_.resize(samples.size());
    thrust::transform(thrust::device_pointer_cast(samples.begin()),
                      thrust::device_pointer_cast(samples.end()),
                      thrust::device_pointer_cast(keys_.begin()),
                      Key(this->bin_count(), bounds_.lower, bounds_.upper, margin_));

    //Key key(this->bin_count(), bounds_.lower, bounds_.upper, margin_);
    //unsigned int currentBin = 0xffffffff;
    //HostVector<SimSample2D> s = samples;
    //for(unsigned int i = 0; i < s.size(); i++) {
    //    auto v = s[i];
    //    unsigned int k = key(v);
    //    if(k != currentBin) {
    //        std::cout << "Bin : " << k << ", "
    //                  << i << ", "
    //                  << v.travel() << std::endl;
    //        currentBin = k;
    //    }
    //}

    //unsigned int firstIndex = 0, lastIndex = 0;
    //for(; firstIndex < s.size(); firstIndex++) {
    //    if(s[firstIndex].travel() >= bounds_.lower) {
    //        std::cout << "first travel : " << s[firstIndex].travel() << std::endl;
    //        break;
    //    }
    //}
    //for(lastIndex = firstIndex; lastIndex < s.size(); lastIndex++) {
    //    if(s[lastIndex].travel() > bounds_.upper) {
    //        std::cout << "last travel : " << s[lastIndex].travel() << std::endl;
    //        break;
    //    }
    //}
    //std::cout << "first index : " << firstIndex << std::endl;
    //std::cout << "last  index : " << lastIndex  << std::endl;
    //std::cout << "total       : " << (lastIndex - firstIndex)
    //          << '/' << samples.size() << std::endl;
}

void Binner::compute_keys(const rtac::cuda::DeviceVector<SimSample3D>& samples)
{
    keys_.resize(samples.size());
    thrust::transform(thrust::device_pointer_cast(samples.begin()),
                      thrust::device_pointer_cast(samples.end()),
                      thrust::device_pointer_cast(keys_.begin()),
                      Key(this->bin_count(), bounds_.lower, bounds_.upper, margin_));
}

} //namespace simulation
} //namespace rtac
