#ifndef _DEF_RTAC_SIMULATION_WAVE_SAMPLES_H_
#define _DEF_RTAC_SIMULATION_WAVE_SAMPLES_H_

#include <rtac_base/cuda_defines.h>
#include <rtac_base/cuda/utils.h>

#include <rtac_base/types/VectorView.h>

namespace rtac  { namespace simulation {

// These types define interfaces for a sample type to be used in
// rtac_simulation.

template <class Derived, typename T>
struct WaveSample
{
    RTAC_HOSTDEVICE rtac::cuda::Complex<T>& sample() {
        return reinterpret_cast<Derived*>(this)->sample();
    }
    RTAC_HOSTDEVICE rtac::cuda::Complex<T>& sample() const {
        return reinterpret_cast<const Derived*>(this)->sample();
    }
};

template <class Derived, typename T>
struct RangedSample : public WaveSample<Derived, T>
{
    RTAC_HOSTDEVICE T& range() {
        return reinterpret_cast<Derived*>(this)->range();
    }
    RTAC_HOSTDEVICE T range() const {
        return reinterpret_cast<const Derived*>(this)->range();
    }
};

template <class Derived, typename T>
struct PolarSample2D : public RangedSample<Derived, T>
{
    RTAC_HOSTDEVICE T& bearing() {
        return reinterpret_cast<Derived*>(this)->bearing();
    }
    RTAC_HOSTDEVICE T bearing() const {
        return reinterpret_cast<const Derived*>(this)->bearing();
    }
};

template <class Derived, typename T>
struct PolarSample3D : public PolarSample2D<Derived, T>
{
    RTAC_HOSTDEVICE T& elevation() {
        return reinterpret_cast<Derived*>(this)->elevation();
    }
    RTAC_HOSTDEVICE T elevation() const {
        return reinterpret_cast<const Derived*>(this)->elevation();
    }
};

}; //namespace simulation
}; //namespace rtac

#endif //_DEF_RTAC_SIMULATION_WAVE_SAMPLES_H_
