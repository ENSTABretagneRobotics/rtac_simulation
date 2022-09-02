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
 * represented by a complex response and a 2D location.
 *
 * The API of the rtac_simulation package is fully templated and the user can
 * replace this type with one of its own. However, some contrains have to be
 * respected to be fully compatible with the rtac_simulation API:
 * - Type must be an aggregate (https://en.cppreference.com/w/cpp/language/aggregate_initialization)
 * - datum must be compatible with std::complex<T> (and thrust::complex<T>)
 * - Must implement operator*= and operator* at least with float and float2
 *   types (float2 type is to be considered a complex number).
 */
template <typename T>
struct Sample2D
{
    using Scalar   = T;
    using Datum    = rtac::cuda::Complex<T>;

    rtac::cuda::Complex<T> datum;
    T x;
    T y;

    Sample2D<T>& operator*=(T a) {
        datum *= a;
        return *this;
    }

    Sample2D<T>& operator*=(float2 a) {
        datum = Datum{datum.real()*a.x - datum.imag()*a.y,
                      datum.real()*a.y + datum.imag()*a.x};
        return *this;
    }
};

}; //namespace simulation
}; //namespace rtac

template <typename T, typename T2>
inline rtac::simulation::Sample2D<T> operator*(const rtac::simulation::Sample2D<T>& s, T2 a)
{
    rtac::simulation::Sample2D<T> res(s);
    res *= a;
    return res;
}
template <typename T, typename T2>
inline rtac::simulation::Sample2D<T> operator*(T2 a, const rtac::simulation::Sample2D<T>& s)
{
    return s * a;
}

template <typename T>
inline std::ostream& operator<<(std::ostream& os, const rtac::simulation::Sample2D<T>& s)
{
    os << "(" << s.datum().real() << ", " << s.datum.imag << "),("
       << s.x << ", " << s.y << ")";
    return os;
}

#endif //_DEF_RTAC_SIMULATION_SAMPLE_H_
