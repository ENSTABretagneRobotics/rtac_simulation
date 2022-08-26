#ifndef _DEF_RTAC_SIMULATION_SAMPLE_H_
#define _DEF_RTAC_SIMULATION_SAMPLE_H_

#include <iostream>
#include <array>

#include <rtac_base/cuda/utils.h>

namespace rtac { namespace simulation {

/**
 * This is the preferred way to represent a simulation sample.
 *
 * For acoustic simulation, each contribution to the simulated sensor output is
 * represented by a complex response and a location. The location can be of any
 * dimension, but dimension 2 (range + bearing) is preferred for GPU
 * performance reasons.
 *
 * The API of the rtac_simulation package is fully templated and the user can
 * replace this type with one of its own. However, some contrains have to be
 * respected to be fully compatible with the rtac_simulation API:
 * - Type must be an aggregate (https://en.cppreference.com/w/cpp/language/aggregate_initialization)
 * - The getters/setters datum() and location() must be implemented
 * - Datum must be compatible with std::complex<T> (and thrust::complex<T>)
 * - Must implement operator*= and operator* at least with float and float2
 *   types (float2 type is to be considered a complex number).
 */
template <typename T, unsigned int LocDimV = 2>
struct Sample
{
    using Datum    = rtac::cuda::Complex<T>;
    using Scalar   = T;
    using Location = std::array<T, LocDimV>;
    static constexpr unsigned int LocDimension = LocDimV;

    rtac::cuda::Complex<T> datum_;
    std::array<T,LocDimV>  location_;

    RTAC_HOSTDEVICE Datum     datum()    const { return datum_; }
    RTAC_HOSTDEVICE Datum&    datum()          { return datum_; }
    RTAC_HOSTDEVICE Location  location() const { return location_; }
    RTAC_HOSTDEVICE Location& location()       { return location_; }

    Sample<T,LocDimV>& operator*=(T a) {
        datum_ *= a;
        return *this;
    }

    Sample<T,LocDimV>& operator*=(float2 a) {
        datum_ = Datum{datum_.real()*a.x - datum_.imag()*a.y,
                       datum_.real()*a.y + datum_.imag()*a.x};
        return *this;
    }
};

}; //namespace simulation
}; //namespace rtac

template <typename T, unsigned int D, typename T2>
inline rtac::simulation::Sample<T,D> operator*(const rtac::simulation::Sample<T,D>& s, T2 a)
{
    rtac::simulation::Sample<T,D> res(s);
    res *= a;
    return res;
}
template <typename T, unsigned int D, typename T2>
inline rtac::simulation::Sample<T,D> operator*(T2 a, const rtac::simulation::Sample<T,D>& s)
{
    return s * a;
}

template <typename T, unsigned int D>
inline std::ostream& operator<<(std::ostream& os, const rtac::simulation::Sample<T,D>& s)
{
    os << "(" << s.datum().real() << ", " << s.datum.imag << "),(";
    
    os << s.location()[0];
    for(int i = 1; i < D; i++) {
        os << ", " << s.location()[i];
    }
    os << ")";
    return os;
}

#endif //_DEF_RTAC_SIMULATION_SAMPLE_H_
