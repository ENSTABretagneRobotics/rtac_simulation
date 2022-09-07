#ifndef _DEF_RTAC_SIMULATION_SAMPLE_H_
#define _DEF_RTAC_SIMULATION_SAMPLE_H_

#include <iostream>
#include <array>

#include <rtac_simulation/common.h>

namespace rtac { namespace simulation {

/**
 * This is the preferred way to represent a simulation sample.
 *
 * For acoustic simulation, each contribution to the simulated sensor output is
 * represented by a complex response and a 2D location.
 *
 * The PointT type is expected to be a CUDA vector type such as float2/3/4. It
 * represents a 3D location in space expressed by default in cartesian
 * coordinates (but are used as poler coordinates in some conditions).
 *
 * The API of the rtac_simulation package is fully templated and the user can
 * replace this type with one of its own. However, some contrains have to be
 * respected to be fully compatible with the rtac_simulation API:
 * - Type must be an aggregate (https://en.cppreference.com/w/cpp/language/aggregate_initialization)
 * - datum must be compatible with std::complex<T> (and thrust::complex<T>)
 * - Must implement operator*= and operator* at least with float and float2
 *   types (float2 type is to be considered a complex number).
 */
template <typename T, typename P>
struct Sample
{
    using Scalar = T;
    using Point  = P;

    Complex<T> datum;
    Point      position;

    Sample<T,P>& operator*=(T a) {
        datum *= a;
        return *this;
    }

    Sample<T,P>& operator*=(float2 a) {
        datum = Complex<T>{datum.real()*a.x - datum.imag()*a.y,
                           datum.real()*a.y + datum.imag()*a.x};
        return *this;
    }
};

}; //namespace simulation
}; //namespace rtac

template <typename T, typename P, typename T2>
inline rtac::simulation::Sample<T,P> operator*(const rtac::simulation::Sample<T,P>& s, T2 a)
{
    rtac::simulation::Sample<T,P> res(s);
    res *= a;
    return res;
}
template <typename T, typename P, typename T2>
inline rtac::simulation::Sample<T,P> operator*(T2 a, const rtac::simulation::Sample<T,P>& s)
{
    return s * a;
}

template <typename T, typename P>
inline std::ostream& operator<<(std::ostream& os, const rtac::simulation::Sample<T,P>& s)
{
    os << "(" << s.datum.real() << ", " << s.datum.imag << "),("
       << s.position << ")";
    return os;
}

#endif //_DEF_RTAC_SIMULATION_SAMPLE_H_
