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
    binIndexes_.resize(binCount_);
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

__global__ void do_make_segments(VectorView<const unsigned int> keys,
                                 uint2* binIndexes)
{
    auto tid = blockDim.x*blockIdx.x + threadIdx.x;
    if(tid < keys.size() && keys[tid] != Binner::Key::OutOfRange) {
        if(tid == 0) {
            binIndexes[0].x = 0;
        }
        else if(keys[tid - 1] != keys[tid]) {
            binIndexes[keys[tid]].x = tid;
        }
        if(tid == keys.size() - 1) {
            binIndexes[keys[tid]].y = keys.size();
        }
        else if(keys[tid] != keys[tid + 1]) {
            binIndexes[keys[tid]].y = tid + 1;
        }
    }
}

void Binner::make_segments()
{
    cudaMemset(binIndexes_.begin(), 0, sizeof(uint2)*binIndexes_.size());

    static constexpr unsigned int BlockSize = 256;
    do_make_segments<<<(keys_.size() - 1) / BlockSize + 1, BlockSize>>>(keys_.const_view(),
                                                                        binIndexes_.begin());
    cudaDeviceSynchronize();
    CUDA_CHECK_LAST();
}

template <typename T>
__global__ void do_make_bins(VectorView<const T>* bins,
                             VectorView<const uint2> binIndexes,
                             const T* samples,
                             int binOverlap)
{
    int tid = blockDim.x*blockIdx.x + threadIdx.x;
    if(tid < binIndexes.size()) {
        int istart = max(0, (int)tid - binOverlap);
        int iend   = min((int)binIndexes.size() - 1, tid + binOverlap);

        for(; istart < iend && binIndexes[istart].y == binIndexes[istart].x; istart++);
        for(; iend > istart && binIndexes[iend].y   == binIndexes[iend].x;   iend--);

        if(istart >= iend) {
            bins[tid] = VectorView<const T>(0, nullptr);
        }
        else {
            bins[tid] = VectorView<const T>(binIndexes[iend].y - binIndexes[istart].x,
                                            samples + binIndexes[istart].x);
        }
        //bins[tid] = VectorView<const SimSample2D>(binIndexes[tid].y - binIndexes[tid].x,
        //                                          samples + binIndexes[tid].x);
    }
}

void Binner::make_bins(rtac::cuda::DeviceVector<rtac::VectorView<const SimSample1D>>& bins,
                       const rtac::cuda::DeviceVector<SimSample1D>& samples)
{
    bins.resize(this->bin_count());
    float resolution = bounds_.length() / (this->bin_count() - 1);
    int binOverlap = margin_ / resolution;

    static constexpr unsigned int BlockSize = 256;
    do_make_bins<<<(this->bin_count() - 1) / BlockSize + 1, BlockSize>>>(
        bins.begin(), binIndexes_.const_view(), samples.begin(), binOverlap);
    cudaDeviceSynchronize();
    CUDA_CHECK_LAST();
}

void Binner::make_bins(rtac::cuda::DeviceVector<rtac::VectorView<const SimSample2D>>& bins,
                       const rtac::cuda::DeviceVector<SimSample2D>& samples)
{
    bins.resize(this->bin_count());
    float resolution = bounds_.length() / (this->bin_count() - 1);
    int binOverlap = margin_ / resolution;

    //std::cout << "bin overlap : " << binOverlap << std::endl;

    static constexpr unsigned int BlockSize = 256;
    do_make_bins<<<(this->bin_count() - 1) / BlockSize + 1, BlockSize>>>(
        bins.begin(), binIndexes_.const_view(), samples.begin(), binOverlap);
    cudaDeviceSynchronize();
    CUDA_CHECK_LAST();
}

} //namespace simulation
} //namespace rtac
