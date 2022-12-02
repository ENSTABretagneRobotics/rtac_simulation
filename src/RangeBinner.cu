#include <rtac_simulation/RangeBinner.h>

#include <thrust/device_ptr.h>
#include <thrust/sort.h>
#include <thrust/execution_policy.h>

namespace rtac { namespace simulation {

RangeBinner::RangeBinner() :
    binCount_(0), rangeMin_(0.0f), rangeMax_(1.0f)
{
    keyLUT_.set_read_mode(cuda::Texture2D<uint32_t>::ReadElementType, false);
    keyLUT_.set_filter_mode(cuda::Texture2D<uint32_t>::FilterNearest, false);
    keyLUT_.set_wrap_mode(cuda::Texture2D<uint32_t>::WrapBorder,
                          cuda::Texture2D<uint32_t>::WrapClamp, false);
    keyLUT_.set_border_color(reinterpret_cast<const float&>(OutOfRangeIndex),
                             0,0,0, true);
}

RangeBinner::RangeBinner(float rangeMin, float rangeMax, uint32_t binCount) :
    RangeBinner()
{
    this->update_ranges(rangeMin, rangeMax, binCount);
}

void RangeBinner::update_ranges(float rangeMin, float rangeMax, uint32_t binCount)
{
    if(rangeMin == rangeMin_ && rangeMax == rangeMax_ && binCount == binCount_)
        return;

    if(rangeMax - rangeMin < 1.0e-6) {
        throw std::runtime_error("RangeBinner : Invalid range bounds");
    }

    std::vector<uint32_t> binIndexes(binCount);
    for(int i = 0; i < binCount; i++) binIndexes[i] = i;
    keyLUT_.set_image(binCount, 1, binIndexes.data());

    rangeMin_ = rangeMin;
    rangeMax_ = rangeMax;
    binCount_ = binCount;
}

template <>
void RangeBinner::compute_keys<PolarSample2D<float>>(
    const cuda::DeviceVector<PolarSample2D<float>>& rangedData) const
{
    keys_.resize(rangedData.size());
    
    float a = 1.0f / (rangeMax_ - rangeMin_);
    float b = -rangeMin_ * a;
    details::binner_compute_keys<<<(rangedData.size() / RTAC_BLOCKSIZE) + 1, RTAC_BLOCKSIZE>>>(
        KeyProcessor{a, b, keyLUT_.texture()}, rangedData.view(), keys_.view());
    cudaDeviceSynchronize();
}

template <>
void RangeBinner::compute_bins<PolarSample2D<float>>(
        cuda::DeviceVector<PolarSample2D<float>>& rangedData,
        cuda::HostVector<VectorView<PolarSample2D<float>>>& bins,
        int overlap) const 
{
    this->compute_keys(rangedData);
    thrust::sort_by_key(thrust::device,
                        thrust::device_pointer_cast(keys_.begin()),
                        thrust::device_pointer_cast(keys_.end()),
                        thrust::device_pointer_cast(rangedData.begin()));
    this->build_bins(rangedData, bins, overlap);
}

} //namespace simulation
} //namespace rtac
