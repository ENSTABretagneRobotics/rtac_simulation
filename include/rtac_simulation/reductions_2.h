#ifndef _DEF_RTAC_SIMULATION_REDUCTIONS_2_H_
#define _DEF_RTAC_SIMULATION_REDUCTIONS_2_H_

#include <rtac_base/cuda_defines.h>
#include <rtac_base/containers/VectorView.h>
#include <rtac_base/containers/ScaledImage.h>
#include <rtac_base/cuda/DeviceVector.h>

#include <rtac_simulation/ReductionKernel.h>
#include <rtac_simulation/Sample.h>

namespace rtac { namespace simulation {

template <typename T, typename KT> class SensorModel2D;

void sparse_convolve_2d(SensorModel2D<Complex<float>, float>& out,
                        const cuda::DeviceVector<VectorView<const SimSample2D>>& bins);

} //namespace simulation
} //namespace rtac


#endif //_DEF_RTAC_SIMULATION_REDUCTIONS_H_