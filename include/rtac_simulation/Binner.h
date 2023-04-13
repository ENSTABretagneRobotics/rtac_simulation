#ifndef _DEF_RTAC_SIMULATION_BINNER_H_
#define _DEF_RTAC_SIMULATION_BINNER_H_

#include <limits>

#include <rtac_base/types/Bounds.h>
#include <rtac_base/types/Linspace.h>
#include <rtac_base/containers/HostVector.h>
#include <rtac_base/cuda/CudaVector.h>

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
    rtac::cuda::CudaVector<unsigned int> keys_;
    rtac::cuda::CudaVector<uint2> binIndexes_; // x is bin start, y is bin end

    void compute_keys(const rtac::cuda::CudaVector<SimSample1D>& samples);
    void compute_keys(const rtac::cuda::CudaVector<SimSample2D>& samples);
    void compute_keys(const rtac::cuda::CudaVector<SimSample3D>& samples);
    void make_segments();

    void make_bins(rtac::cuda::CudaVector<rtac::VectorView<const SimSample1D>>& bins,
                   const rtac::cuda::CudaVector<SimSample1D>& samples);
    void make_bins(rtac::cuda::CudaVector<rtac::VectorView<const SimSample2D>>& bins,
                   const rtac::cuda::CudaVector<SimSample2D>& samples);
    //void make_bins(rtac::cuda::CudaVector<rtac::VectorView<const SimSample3D>>& bins,
    //               const rtac::cuda::CudaVector<SimSample3D>& samples);
    template <typename T>
    void compute_bins(rtac::cuda::CudaVector<rtac::VectorView<const T>>& bins,
                      const rtac::cuda::CudaVector<T>& samples);

    public:

    Binner() = default;
    Binner(unsigned int binCount, const Bounds<float>& bounds,
           float margin = 0.0f);

    void reconfigure(unsigned int binCount, const Bounds<float>& bounds,
                     float margin = 0.0f);
    void reconfigure(const Linspace<float>& bins, float margin = 0.0f) {
        this->reconfigure(bins.size(), bins.bounds(), margin);
    }

    unsigned int         bin_count() const { return binCount_; }
    const Bounds<float>& bounds()    const { return bounds_;   }
    float                margin()    const { return margin_;   }

    // input data is supposed to be sorted
    template <typename T>
    void compute(rtac::cuda::CudaVector<rtac::VectorView<const T>>& bins,
                 const rtac::cuda::CudaVector<T>& samples)
    {
        this->compute_keys(samples);
        this->compute_bins(bins, samples);
    }
};

/**
 * This method is not GPUised but should be
 */
template <typename T>
void Binner::compute_bins(rtac::cuda::CudaVector<rtac::VectorView<const T>>& bins,
                          const rtac::cuda::CudaVector<T>& samples)
{
    this->make_segments();
    this->make_bins(bins, samples);

    //HostVector<unsigned int> keys = keys_;
    //std::vector<const T*>     binStarts(this->bin_count(), nullptr);
    //std::vector<unsigned int> binSizes(this->bin_count(),  0);
    //HostVector<rtac::VectorView<const T>> tmpBins(this->bin_count());

    //for(unsigned int i = 0; i < keys.size(); i++) {
    //    if(keys[i] == Key::OutOfRange) continue;
    //    if(binStarts[keys[i]] == nullptr) {
    //        binStarts[keys[i]] = samples.begin() + i;
    //    }
    //    binSizes[keys[i]]++;
    //}
    //HostVector<VectorView<const T>> tmp(bins);
    //HostVector<uint2>               idx(binIndexes_);

    //for(unsigned int i = 0; i < tmpBins.size(); i++) {
    //    tmpBins[i] = VectorView<const T>(binSizes[i], binStarts[i]);
    //    //std::cout << "bin0 : " << i << ", " << binStarts[i] - samples.begin()
    //    //          << ", " << binSizes[i] << std::endl;
    //    std::cout << "idx : " << i << ", " << idx[i].x
    //              << ", " << idx[i].y - idx[i].x
    //              << ", " << idx[i].x << ", " << idx[i].y << std::endl;
    //    std::cout << "bin : " << i << ", " << tmp[i].data() - samples.data()
    //              << ", " << tmp[i].size() << std::endl << std::endl;
    //}
    ////bins = tmpBins;
}


} //namespace simulation
} //namespace rtac

inline std::ostream& operator<<(std::ostream& os, const rtac::simulation::Binner& binner)
{
    os << "Binner : " << binner.bounds() << ", " << binner.bin_count()
       << " (margin : " << binner.margin() << ")";
    return os;
}

#endif //_DEF_RTAC_SIMULATION_BINNER_H_
