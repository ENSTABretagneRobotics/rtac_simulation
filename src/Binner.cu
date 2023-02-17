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
