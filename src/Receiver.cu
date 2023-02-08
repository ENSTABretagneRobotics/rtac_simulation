#include <rtac_simulation/Receiver.h>

#include <thrust/device_ptr.h>
#include <thrust/sort.h>
#include <thrust/execution_policy.h>

namespace rtac { namespace simulation {


struct Lesser
{
   __device__ __host__ bool operator()(const SimSample1D& lhs,
                                        const SimSample1D& rhs) const
   {
       return lhs.travel() <= rhs.travel();
   }
    __device__ __host__ bool operator()(const SimSample2D& lhs,
                                        const SimSample2D& rhs) const
   {
       return lhs.travel() <= rhs.travel();
   }
   __device__ __host__ bool operator()(const SimSample3D& lhs,
                                        const SimSample3D& rhs) const
   {
       return lhs.travel() <= rhs.travel();
   }
};


void sort(rtac::cuda::DeviceVector<SimSample1D>& samples)
{
    thrust::sort(thrust::device,
                 thrust::device_pointer_cast(samples.begin()),
                 thrust::device_pointer_cast(samples.end()),
                 Lesser());
}

void sort(rtac::cuda::DeviceVector<SimSample2D>& samples)
{
    thrust::sort(thrust::device,
                 thrust::device_pointer_cast(samples.begin()),
                 thrust::device_pointer_cast(samples.end()),
                 Lesser());
}

void sort(rtac::cuda::DeviceVector<SimSample3D>& samples)
{
    thrust::sort(thrust::device,
                 thrust::device_pointer_cast(samples.begin()),
                 thrust::device_pointer_cast(samples.end()),
                 Lesser());
}


} //namespace simulation
} //namespace rtac
