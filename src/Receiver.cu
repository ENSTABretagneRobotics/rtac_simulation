#include <rtac_simulation/Receiver.h>

#include <thrust/device_ptr.h>
#include <thrust/sort.h>
#include <thrust/execution_policy.h>

namespace rtac { namespace simulation {

template <class S>
struct Lesser
{
   __device__ __host__ bool operator()(const Sample1D<S>& lhs,
                                       const Sample1D<S>& rhs) const
   {
       return lhs.travel() <= rhs.travel();
   }
};


void sort(rtac::cuda::DeviceVector<SimSample1D>& samples)
{
    thrust::sort(thrust::device,
                 thrust::device_pointer_cast(samples.begin()),
                 thrust::device_pointer_cast(samples.end()),
                 Lesser<SimSample1D>());
}

void sort(rtac::cuda::DeviceVector<SimSample2D>& samples)
{
    thrust::sort(thrust::device,
                 thrust::device_pointer_cast(samples.begin()),
                 thrust::device_pointer_cast(samples.end()),
                 Lesser<SimSample2D>());
}

void sort(rtac::cuda::DeviceVector<SimSample3D>& samples)
{
    thrust::sort(thrust::device,
                 thrust::device_pointer_cast(samples.begin()),
                 thrust::device_pointer_cast(samples.end()),
                 Lesser<SimSample3D>());
}

} //namespace simulation
} //namespace rtac
