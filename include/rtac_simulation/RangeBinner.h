#ifndef _DEF_RTAC_SIMULATION_RANGE_BINNER_H_
#define _DEF_RTAC_SIMULATION_RANGE_BINNER_H_

#include <iostream>
#include <memory>

#include <rtac_base/cuda/Texture2D.h>

#include <rtac_simulation/common.h>
#include <rtac_simulation/Sample.h>

namespace rtac { namespace simulation {

/**
 * This performs binning on a set of data (only linearly spaced bin supported
 * for now).
 */
class RangeBinner
{
    public:

    using Ptr      = std::shared_ptr<RangeBinner>;
    using ConstPtr = std::shared_ptr<const RangeBinner>;

    static constexpr uint32_t OutOfRangeIndex = 0xffffffff;

    struct KeyProcessor {
        const float         a_;
        const float         b_;
        cudaTextureObject_t lut_;
        #ifdef RTAC_CUDACC
        __device__ uint32_t key(float range) const {
            // fmaf performs a_*range + b_ in a single instruction. The texture
            // fetch (supposedly) allow of fast out of bounds check.
            //printf("bin value : %f\n", fmaf(a_, range, b_));
            return tex2D<uint32_t>(lut_, fmaf(a_, range, b_), 0.0f);
        }
        #endif
    };

    protected:

    rtac::cuda::Texture2D<uint32_t> keyLUT_;
    uint32_t binCount_;
    float    rangeMin_;
    float    rangeMax_;

    mutable DeviceVector<uint32_t> keys_;
    mutable HostVector<uint32_t>   tmpKeys_;
    
    RangeBinner();
    RangeBinner(float rangeMin, float rangeMax, uint32_t binCount);

    template <typename T>
    void compute_keys(const DeviceVector<T>& rangedData) const;
    template <typename T>
    void build_bins(DeviceVector<T>& rangedData,
                    HostVector<VectorView<T>>& bins) const;

    public:

    static Ptr Create() { return Ptr(new RangeBinner()); }
    static Ptr Create(float rangeMin, float rangeMax, uint32_t binCount) {
        return Ptr(new RangeBinner(rangeMin, rangeMax, binCount));
    }

    void update_ranges(float rangeMin, float rangeMax, uint32_t binCount);
    
    template <typename T>
    void compute_bins(DeviceVector<T>& rangedData,
                      HostVector<VectorView<T>>& bins) const;
    template <typename T>
    DeviceVector<VectorView<T>> compute_bins(DeviceVector<T>& rangedData) const {
        HostVector<VectorView<T>> bins;
        this->compute_bins(rangedData, bins);
        return bins;
    }
};

template <typename T>
void RangeBinner::build_bins(DeviceVector<T>& rangedData,
                             HostVector<VectorView<T>>& bins) const
{
    // reinitializing bins to ensure empty bins have a zeroed size.
    bins.resize(binCount_);
    for(auto& bin : bins) bin = VectorView<T>{0,nullptr};

    tmpKeys_ = keys_;

    uint32_t currentBin     = 0;
    uint32_t currentBinStart = 0;
    int i = 0;
    for(; i < tmpKeys_.size(); i++) {
        if(tmpKeys_[i] > currentBin) {
            std::cout << "current bin : " << currentBin << "/" << binCount_ << std::endl;
            bins[currentBin] = VectorView<T>{i - currentBinStart,
                                             rangedData.data() + currentBinStart};
            currentBin = tmpKeys_[i];
            currentBinStart = i;
        }
        // keys are sorted in ascending order. If the key is above binCount,
        // all of remaining samples are out of range.
        if(currentBin >= binCount_) break;
    }
    if(currentBin < binCount_) {
        // here this means we did not have any invalid samples in the set.
        // The last bin was not finalized in the last loop.
        bins[currentBin] = VectorView<T>{i - currentBinStart,
                                         rangedData.data() + currentBinStart};
    }
}

namespace details {

#ifdef RTAC_CUDACC

template <typename T>
__global__ void binner_compute_keys(RangeBinner::KeyProcessor keyProc,
                                    VectorView<const T> rangedData,
                                    VectorView<uint32_t> keys)
{
    auto idx = blockDim.x*blockIdx.x + threadIdx.x;
    for(; idx < rangedData.size(); idx += gridDim.x*blockDim.x) {
        keys[idx] = keyProc.key(rangedData[idx].range());
    }
}

#endif //RTAC_CUDACC

} //namespace details


} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_RANGE_BINNER_H_
