#ifndef _DEF_RTAC_SIMULATION_SENSOR_MODEL_H_
#define _DEF_RTAC_SIMULATION_SENSOR_MODEL_H_

#include <rtac_base/containers/ScaledImage.h>
#include <rtac_base/cuda/DeviceVector.h>

#include <rtac_simulation/ReductionKernel.h>
#include <rtac_simulation/Binner.h>

namespace rtac { namespace simulation {

template <typename T, 
          class BDimT,
          typename KT = T>
class SensorModel2D
{
    public:

    using Ptr      = std::shared_ptr<SensorModel2D<T,BDimT,KT>>;
    using ConstPtr = std::shared_ptr<const SensorModel2D<T,BDimT,KT>>;

    using DataImage = ScaledImage<T, BDimT, cuda::DeviceVector>;

    using BearingDimension = BDimT;
    //using RangeDimension   = RDimT;

    protected:

    DataImage    data_;
    Kernel2D<KT> pointSpreadFunction_;
    Binner       binner_;

    SensorModel2D(const BDimT& widthDim,
                  const RDimT& heightDim,
                  const Kernel2D<KT>& psf) :
        data_(widthDim, heightDim),
        pointSpreadFunction_(psf)
    {}

    void compute_bins(const DeviceVector<SimSample2D>& samples)
    {
        keys_->resize(samples.size());
    }

    public:

    static Ptr Create(const BDimT& widthDim,
                      const RDimT& heightDim,
                      const Kernel2D<KT>& psf)
    {
        return Ptr(new SensorModel2D<T,BDimT,RDimT,T>(widthDim, heightDim, psf));
    }
    
    const DataImage& data() const { return data_; }
          //DataImage& data()       { return data_; }

    const Kernel2D<KT>& point_spread_function() const { return pointSpreadFunction_; }
          //Kernel2D<KT>& point_spread_function()       { return pointSpreadFunction_; }
    
    template <typename T2>
    void reduce_samples(const DeviceVector<T2>& samples)
    {
        rtac::cuda::DeviceVector<rtac::VectorView<const T2>> bins;
        binner.compute(bins, samples);
    }
};

/**
 * This a factory class to easily create SensorModel types. It is mostly there
 * to take advantage of the type deduction system.
 */
template <typename T>
struct SensorModel
{
    template <class BDimT, class RDimT, typename KT = T>
    static SensorModel2D<T,BDimT,RDimT,KT>::Ptr make(const BDimT& widthDim,
                                                     const RDimT& heightDim,
                                                     const Kernel2D<KT>& kernel)
    {
        return SensorModel2D<T,BDimT,RDimT,KT>::Create(widthDim, heightDim, kernel);
    }
};

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_SENSOR_MODEL_H_
