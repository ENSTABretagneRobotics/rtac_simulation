#ifndef _DEF_RTAC_SIMULATION_HELPERS_RECEIVER_FACTORIES_H_
#define _DEF_RTAC_SIMULATION_HELPERS_RECEIVER_FACTORIES_H_

#include <iostream>
#include <vector>

#include <rtac_base/signal_helpers.h>

#include <rtac_simulation/PolarKernel2D.h>

namespace rtac { namespace simulation {

template <typename T>
typename PolarKernel2D<T>::Ptr simple_polar_kernel(float bearingResolution,
                                                   float pulseLength, float waveLength,
                                                   unsigned int oversampling = 8)
{
    signal::SincFunction<float> bearingPsf(oversampling);
    float bearingSpan = bearingPsf.physical_span(bearingResolution * M_PI / 180.0f);

    float numPeriod = pulseLength / waveLength;
    signal::SineFunction<float> rangePsf(numPeriod, oversampling);

    unsigned int W = bearingPsf.size();
    unsigned int H = rangePsf.size();

    Image<T> kernelData({W,H});
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
    signal::SincFunction<float> bearingPsf(bearingSpan / bearingResolution, oversampling);
    //float bearingSpan = bearingPsf.physical_span(bearingResolution * M_PI / 180.0f);

    float numPeriod = pulseLength / waveLength;
    signal::SineFunction<float> rangePsf(numPeriod, oversampling);

    unsigned int W = bearingPsf.size();
    unsigned int H = rangePsf.size();

    Image<T> kernelData(W,H);
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


