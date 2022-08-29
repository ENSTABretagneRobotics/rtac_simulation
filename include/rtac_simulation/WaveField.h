#ifndef _DEF_RTAC_SIMULATION_WAVE_FIELD_H_
#define _DEF_RTAC_SIMULATION_WAVE_FIELD_H_

#include <iostream>

#include <rtac_base/cuda_defines.h>
#include <rtac_base/types/VectorView.h>
#include <rtac_base/cuda/utils.h>

namespace rtac { namespace simulation {

template<typename T, template<typename>class VectorT>
class WaveField
{
    public:

    using WaveSample = rtac::cuda::Complex<T>;
    template <typename T0>
    using Vector     = VectorT<T0>;

    protected:
    
    Vector<WaveSample> samples_;
    Vector<T>          x_;
    Vector<T>          y_;
    Vector<T>          z_;

    public:

    WaveField() {}
    WaveField(std::size_t size) { this->resize(size); }
    WaveField(const Vector<WaveSample>& samples,
              const Vector<T>& x, const Vector<T>& y, const Vector<T>& z);
    template <template<typename>class VectorT2>
    WaveField(const WaveField<T, VectorT2>& other) { *this = other; }
    template <template<typename>class VectorT2>
    WaveField<T,VectorT>& operator=(const WaveField<T,VectorT2>& other);

    void resize(std::size_t size);
    void clear();
    RTAC_HOSTDEVICE std::size_t size()     const { return samples_.size(); }
    RTAC_HOSTDEVICE std::size_t capacity() const { return samples_.capacity(); }

    const Vector<WaveSample>& samples() const { return samples_; }
    Vector<WaveSample>&       samples()       { return samples_; }

    const Vector<T>& x() const { return x_; }
    const Vector<T>& y() const { return y_; }
    const Vector<T>& z() const { return z_; }
    Vector<T>&       x()       { return x_; }
    Vector<T>&       y()       { return y_; }
    Vector<T>&       z()       { return z_; }

    WaveField<const T, rtac::types::VectorView> view() const;
    WaveField<T, rtac::types::VectorView>       view();

    RTAC_HOSTDEVICE WaveSample  sample(std::size_t idx) const { return samples_[idx]; }
    RTAC_HOSTDEVICE WaveSample& sample(std::size_t idx)       { return samples_[idx]; }
    RTAC_HOSTDEVICE T  x(std::size_t idx) const { return x_[idx]; }
    RTAC_HOSTDEVICE T  y(std::size_t idx) const { return y_[idx]; }
    RTAC_HOSTDEVICE T  z(std::size_t idx) const { return z_[idx]; }
    RTAC_HOSTDEVICE T& x(std::size_t idx)       { return x_[idx]; }
    RTAC_HOSTDEVICE T& y(std::size_t idx)       { return y_[idx]; }
    RTAC_HOSTDEVICE T& z(std::size_t idx)       { return z_[idx]; }
};

template <typename T>
using WaveFieldView = WaveField<T, rtac::types::VectorView>;
template <typename T>
using WaveFieldConstView = WaveField<const T, rtac::types::VectorView>;

template <typename T, template<typename>class V>
WaveField<T,V>::WaveField(const Vector<WaveSample>& samples,
                          const Vector<T>& x,
                          const Vector<T>& y,
                          const Vector<T>& z) :
    samples_(samples),
    x_(x), y_(y), z_(z)
{}

template <typename T, template<typename>class V> template<template<typename>class V2>
WaveField<T,V>& WaveField<T,V>::operator=(const WaveField<T,V2>& other)
{
    samples_ = other.samples();
    x_ = other.x();
    y_ = other.y();
    z_ = other.z();

    return *this;
}

template <typename T, template<typename>class V>
void WaveField<T,V>::resize(std::size_t size)
{
    try {
        samples_.resize(size);
        x_.resize(size);
        y_.resize(size);
        z_.resize(size);
    }
    catch(...) {
        this->clear();
        throw; // rethrow original exception
    }
}

template <typename T, template<typename>class V>
void WaveField<T,V>::clear()
{
    samples_.clear();
    x_.clear();
    y_.clear();
    z_.clear();
}

template <typename T, template<typename>class V>
WaveField<const T, rtac::types::VectorView> WaveField<T,V>::view() const
{
    return WaveField<const T, rtac::types::VectorView>(
        samples_.view(), x_.view(), y_.view(), z_.view());
}

template <typename T, template<typename>class V>
WaveField<T, rtac::types::VectorView> WaveField<T,V>::view()
{
    return WaveField<T, rtac::types::VectorView>(
        samples_.view(), x_.view(), y_.view(), z_.view());
}

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_WAVE_FIELD_H_
