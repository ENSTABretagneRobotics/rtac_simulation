#ifndef _DEF_RTAC_SIMULATION_HELPERS_RECEIVER_FACTORIES_H_
#define _DEF_RTAC_SIMULATION_HELPERS_RECEIVER_FACTORIES_H_

#include <iostream>
#include <vector>

#include <rtac_simulation/common.h>
#include <rtac_simulation/PolarKernel2D.h>

namespace rtac { namespace simulation {

/**
 * This generates a set of samples of the sinc function.
 *
 * The function is sampled at a period of oversampling*pi. The returned
 * coefficients make up for roughly 99% of the total energy (integration) of
 * the full sinc function.
 *
 * The most prominent use of this function in the rtac framework involve linear
 * interpolation (texture fetch). The oversampling is therefore important to
 * keep some level of precision.
 *
 * sampling period : pi * oversampling
 * domain          : +- pi * (N - 1) / oversampling, roughly 16*pi
 */
template <typename T>
struct SincFunction
{
    static constexpr T HalfEnergyX = 1.39156;

    protected:

    std::vector<T> x_;
    std::vector<T> y_;
    unsigned int oversampling_;
    T samplingPeriod_;

    public:

    SincFunction(unsigned int oversampling = 8) : 
        x_(16*oversampling),
        y_(x_.size()),
        oversampling_(oversampling),
        samplingPeriod_(M_PI / oversampling)
    {
        auto N = y_.size();
        for(int n = 0; n < N; n++) {
            x_[n] = M_PI * (n - 0.5f*(N - 1)) / oversampling;
            y_[n] = std::sin(x_[n]) / x_[n]; // no need to check for x == 0, never happens
        }
    }

    SincFunction(T span, unsigned int oversampling = 8) : 
        x_(2*((unsigned int)(0.5f*oversampling*span / M_PI) + 1)),
        y_(x_.size()),
        oversampling_(oversampling),
        samplingPeriod_(span / (x_.size() - 1))
    {
        auto N = y_.size();
        for(int n = 0; n < N; n++) {
            x_[n] = span * (((float)n) / (N - 1) - 0.5f);
            y_[n] = std::sin(x_[n]) / x_[n]; // no need to check for x == 0, never happens
        }
    }

    std::size_t size() const { return y_.size();   }
    float sampling_period() const { return samplingPeriod_; }

    const std::vector<T>& domain()   const { return x_; }
    const std::vector<T>& function() const { return y_; }
    
    /**
     * This returns the span of the function domain, given a resolution
     * parameter which corresponds to the domain value x where sin(x) / x =
     * sqrt(0.5)
     */
    T physical_span(T resolution) const {
        T scaling = 2.0 * HalfEnergyX / resolution;
        return 2.0f*x_.back() / scaling;
    }
};

template <typename T>
class SinFunction
{
    protected:

    std::vector<T> x_;
    std::vector<T> y_;
    T              periodCount_;

    public:

    SinFunction(T periodCount, unsigned int oversampling = 8) :
        x_(2*(((unsigned int)periodCount * oversampling) + 1)),
        y_(x_.size()),
        periodCount_(periodCount)

    {
        auto N = x_.size();
        for(int n = 0; n < N; n++) {
            x_[n] = (2.0*M_PI*periodCount_*n) / (N - 1);
            y_[n] = std::sin(x_[n]);
        }
    }

    std::size_t size() const { return y_.size(); }

    const std::vector<T>& phase()    const { return x_; }
    const std::vector<T>& function() const { return y_; }
};

template <typename T>
typename PolarKernel2D<T>::Ptr simple_polar_kernel(float bearingResolution,
                                                   float pulseLength, float waveLength,
                                                   unsigned int oversampling = 8)
{
    SincFunction<float> bearingPsf(oversampling);
    float bearingSpan = bearingPsf.physical_span(bearingResolution * M_PI / 180.0f);

    float numPeriod = pulseLength / waveLength;
    SinFunction<float> rangePsf(numPeriod, oversampling);

    unsigned int W = bearingPsf.size();
    unsigned int H = rangePsf.size();

    Image<T, HostVector> kernelData({W,H});
    for(int h = 0; h < H; h++) {
        for(int w = 0; w < W; w++) {
            kernelData(h,w) = rangePsf.function()[h] * bearingPsf.function()[w];
        }
    }

    return PolarKernel2D<T>::Create(bearingSpan, pulseLength, kernelData);
}

template <typename T>
typename PolarKernel2D<T>::Ptr simple_polar_kernel(float bearingResolution, 
                                                   float bearingSpan,
                                                   float pulseLength,
                                                   float waveLength,
                                                   unsigned int oversampling = 8)
{
    SincFunction<float> bearingPsf(bearingSpan / bearingResolution, oversampling);
    //float bearingSpan = bearingPsf.physical_span(bearingResolution * M_PI / 180.0f);

    float numPeriod = pulseLength / waveLength;
    SinFunction<float> rangePsf(numPeriod, oversampling);

    unsigned int W = bearingPsf.size();
    unsigned int H = rangePsf.size();

    Image<T, HostVector> kernelData({W,H});
    for(int h = 0; h < H; h++) {
        for(int w = 0; w < W; w++) {
            //kernelData(h,w) = rangePsf.function()[h];
            kernelData(h,w) = bearingPsf.function()[w];
            //kernelData(h,w) = rangePsf.function()[h] * bearingPsf.function()[w];
        }
    }

    return PolarKernel2D<T>::Create(bearingSpan * M_PI / 180.0, 
                                    pulseLength, kernelData);
}

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_HELPERS_RECEIVER_FACTORIES_H_


