#ifndef _DEF_RTAC_SIMULATION_BINNER_H_
#define _DEF_RTAC_SIMULATION_BINNER_H_

#include <limits>

#include <rtac_base/types/Bounds.h>

#include <rtac_base/containers/HostVector.h>
#include <rtac_base/cuda/DeviceVector.h>

#include <rtac_simulation/Sample.h>

namespace rtac { namespace simulation {

class Binner
{
    public:

    struct Bin
    {
        unsigned int start;
        unsigned int size;
    };

    struct Key
    {
        static constexpr unsigned int OutOfRange = std::numeric_limits<unsigned int>::max();

        unsigned int binCount_;
        float a_, b_;
        Bounds<float> bounds_; // this includes margins

        Key() = default;
        Key(unsigned int binCount, float minBin, float maxBin, float margin = 0) :
            binCount_(binCount)
        {
            float resolution = (maxBin - minBin) / binCount;
            float m = minBin - 0.5f*resolution;
            float M = maxBin + 0.5f*resolution;

            a_ = (binCount - 1) / (M - m);
            b_ = -m * a_ + 0.5f; // +0.5f for flooring with integer conversion
            bounds_.lower = m - margin;
            bounds_.upper = M + margin;
        }

        template <typename S>
         RTAC_HOSTDEVICE unsigned int operator()(const Sample1D<S>& x) const {
            int res = (int)(fmaf(a_, x.travel(), b_));
            if(res < 0)          return (res >= bounds_.lower) ? 0             : OutOfRange;
            if(res >= binCount_) return (res <= bounds_.upper) ? binCount_ - 1 : OutOfRange;
            return (unsigned int)res;
         }
    };

    protected:

    unsigned int  binCount_;
    Bounds<float> bounds_;
    float         margin_;
    rtac::cuda::DeviceVector<unsigned int> keys_;

    void compute_keys(const rtac::cuda::DeviceVector<SimSample1D>& samples);
    void compute_keys(const rtac::cuda::DeviceVector<SimSample2D>& samples);
    void compute_keys(const rtac::cuda::DeviceVector<SimSample3D>& samples);

    template <typename T>
    void compute_bins(rtac::cuda::DeviceVector<rtac::VectorView<const T>>& bins,
                      const rtac::cuda::DeviceVector<T>& samples);

    public:

    Binner() = default; // remove this ?
    Binner(unsigned int binCount, const Bounds<float>& bounds,
           float margin = 0.0f);

    void reconfigure(unsigned int binCount, const Bounds<float>& bounds,
                     float margin = 0.0f);

    unsigned int         bin_count() const { return binCount_; }
    const Bounds<float>& bounds()    const { return bounds_;   }
    float                margin()    const { return margin_;   }

    // input data is supposed to be sorted
    template <typename T>
    void compute(rtac::cuda::DeviceVector<rtac::VectorView<const T>>& bins,
                 const rtac::cuda::DeviceVector<T>& samples)
    {
        this->compute_keys(samples);
        this->compute_bins(bins, samples);
    }
};

/**
 * This method is not GPUised but should be
 */
template <typename T>
void Binner::compute_bins(rtac::cuda::DeviceVector<rtac::VectorView<const T>>& bins,
                          const rtac::cuda::DeviceVector<T>& samples)
{
    HostVector<unsigned int> keys = keys_;
    HostVector<rtac::VectorView<const T>> tmpBins(this->bin_count());

    unsigned int currentBin = 0;
    const T* binStart       = samples.begin();
    unsigned int binSize    = 0;

    for(unsigned int i = 0; i < keys.size(); i++) {
        // Skipping OutOfRange
        if(keys[i] == Key::OutOfRange) continue;
        while(currentBin < keys[i]) {
            tmpBins[currentBin] = VectorView<const T>(binSize, binStart);
            binStart += binSize;
            binSize   = 0;
            currentBin++;
            if(currentBin >= tmpBins.size()) return;
        }
        if(keys[i] == currentBin) binSize++;
    }
    bins = tmpBins;

    //unsigned int bin = 0;
    //unsigned int sum = 0;
    //unsigned int gSum = 0;
    //for(unsigned int i = 0; i < keys.size(); i++) {
    //    if(keys[i] == Key::OutOfRange) continue;
    //    while(bin < keys[i]) {
    //        //std::cout << "key : " << keys[i] << std::endl;
    //        std::cout << "bin : " << bin << ", " << sum << std::endl;
    //        gSum += sum;
    //        sum = 0;
    //        bin++;
    //        if(bin >= this->bin_count()) {
    //            gSum += sum;
    //            std::cout << "Total sum : " << gSum << '/' << keys_.size() << std::endl;
    //            std::cout << bounds_ << std::endl;
    //            return;
    //        }
    //    }
    //    if(keys[i] == bin) sum++;
    //}
    //gSum += sum;
    //std::cout << "Total sum : " << gSum << '/' << keys_.size() << std::endl;
    //std::cout << bounds_ << std::endl;
}


} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_BINNER_H_
