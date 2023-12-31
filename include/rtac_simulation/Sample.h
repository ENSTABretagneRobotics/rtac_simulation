#ifndef _DEF_RTAC_SIMULATION_SAMPLE_H_
#define _DEF_RTAC_SIMULATION_SAMPLE_H_

#include <iostream>
#include <array>

#include <rtac_base/cuda_defines.h>
#include <rtac_base/types/Complex.h>
#include <rtac_base/cuda/vec_math.h>

namespace rtac { namespace simulation {

//struct RayPayload
//{
//    Complex<float> value_;
//    float          frequency_; // this here ?
//    float          travel_;
//
//    RTAC_HOSTDEVICE RayPayload() = default;
//    RTAC_HOSTDEVICE RayPayload& operator=(const RayPayload&) = default;
//
//    RTAC_HOSTDEVICE RayPayload(const Complex<float>& value, 
//                               float frequency,
//                               float travel = 0.0f) :
//        value_(value), frequency_(frequency), travel_(travel)
//    {}
//
//    RTAC_HOSTDEVICE const Complex<float>& value() const { return value_; }
//    RTAC_HOSTDEVICE       Complex<float>& value()       { return value_; }
//
//    RTAC_HOSTDEVICE const float& frequency() const { return frequency_;    }
//    RTAC_HOSTDEVICE       float& frequency()       { return frequency_;    }
//
//    RTAC_HOSTDEVICE const float& travel() const { return travel_;    }
//    RTAC_HOSTDEVICE       float& travel()       { return travel_;    }
//
//    RTAC_HOSTDEVICE static RayPayload Null()
//    {
//        return RayPayload(Complex<float>(0.0f, 0.0f), 0.0f, -1.0f);
//    }
//};

struct RayPayload
{
    Complex<float> value_;
    float          travel_;

    RTAC_HOSTDEVICE RayPayload() = default;
    RTAC_HOSTDEVICE RayPayload& operator=(const RayPayload&) = default;

    RTAC_HOSTDEVICE RayPayload(const Complex<float>& value, 
                               float travel = 0.0f) :
        value_(value), travel_(travel)
    {}

    RTAC_HOSTDEVICE const Complex<float>& value() const { return value_; }
    RTAC_HOSTDEVICE       Complex<float>& value()       { return value_; }

    RTAC_HOSTDEVICE const float& travel() const { return travel_;    }
    RTAC_HOSTDEVICE       float& travel()       { return travel_;    }

    RTAC_HOSTDEVICE static RayPayload Null()
    {
        return RayPayload(Complex<float>(0.0f, 0.0f), -1.0f);
    }
};

template <class Derived>
struct Sample1D
{
    RTAC_HOSTDEVICE const Derived* cast() const { return static_cast<const Derived*>(this); }
    RTAC_HOSTDEVICE       Derived* cast()       { return static_cast< Derived*>(this);      }

    RTAC_HOSTDEVICE const Complex<float>& value() const { return this->cast()->value(); }
    RTAC_HOSTDEVICE       Complex<float>& value()       { return this->cast()->value(); }

    RTAC_HOSTDEVICE const float& travel() const { return this->cast()->travel(); }
    RTAC_HOSTDEVICE       float& travel()       { return this->cast()->travel(); }
};

template <class Derived>
struct Sample2D : public Sample1D<Derived>
{
    RTAC_HOSTDEVICE const Derived* cast() const { return static_cast<const Derived*>(this); }
    RTAC_HOSTDEVICE       Derived* cast()       { return static_cast< Derived*>(this);      }

    RTAC_HOSTDEVICE const float& bearing() const { return this->cast()->bearing(); }
    RTAC_HOSTDEVICE       float& bearing()       { return this->cast()->bearing(); }
};

template <class Derived>
struct Sample3D_2 : public Sample2D<Derived>
{
    RTAC_HOSTDEVICE const Derived* cast() const { return static_cast<const Derived*>(this); }
    RTAC_HOSTDEVICE       Derived* cast()       { return static_cast< Derived*>(this);      }

    RTAC_HOSTDEVICE const float& elevation() const { return this->cast()->elevation(); }
    RTAC_HOSTDEVICE       float& elevation()       { return this->cast()->elevation(); }
};

struct SimSample1D : public Sample1D<SimSample1D>
{
    Complex<float> value_;
    float          travel_;

    RTAC_HOSTDEVICE const Complex<float>& value() const { return value_; }
    RTAC_HOSTDEVICE       Complex<float>& value()       { return value_; }

    RTAC_HOSTDEVICE const float& travel() const    { return travel_;    }
    RTAC_HOSTDEVICE       float& travel()          { return travel_;    }

    RTAC_HOSTDEVICE static SimSample1D Make(const Complex<float>& value,
                                            float travel,
                                            const float3& dir)
    {
        SimSample1D res;
        res.value_  = value;
        res.travel_ = travel;
        return res;
    }

    RTAC_HOSTDEVICE static SimSample1D Null()
    {
        SimSample1D res{};
        res.travel_ = -1.0f;
        return res;
    }
};

struct SimSample2D : public Sample2D<SimSample2D>
{
    Complex<float> value_;
    float          travel_;
    float          bearing_;

    RTAC_HOSTDEVICE const Complex<float>& value() const { return value_; }
    RTAC_HOSTDEVICE       Complex<float>& value()       { return value_; }

    RTAC_HOSTDEVICE const float& travel() const    { return travel_;    }
    RTAC_HOSTDEVICE       float& travel()          { return travel_;    }
    RTAC_HOSTDEVICE const float& bearing() const   { return bearing_;   }
    RTAC_HOSTDEVICE       float& bearing()         { return bearing_;   }

    RTAC_HOSTDEVICE static SimSample2D Make(const Complex<float>& value,
                                            float travel,
                                            const float3& dir)
    {
        SimSample2D res;
        res.value_   = value;
        res.travel_  = travel;
        res.bearing_ = atan2(-dir.y, dir.x);
        return res;
    }

    RTAC_HOSTDEVICE static SimSample2D Null()
    {
        SimSample2D res{};
        res.travel_ = -1.0f;
        return res;
    }
};

struct SimSample3D : public Sample3D_2<SimSample3D>
{
    Complex<float> value_;
    float          travel_;
    float          bearing_;
    float          elevation_;

    RTAC_HOSTDEVICE const Complex<float>& value() const { return value_; }
    RTAC_HOSTDEVICE       Complex<float>& value()       { return value_; }

    RTAC_HOSTDEVICE const float& travel() const    { return travel_;    }
    RTAC_HOSTDEVICE       float& travel()          { return travel_;    }
    RTAC_HOSTDEVICE const float& bearing() const   { return bearing_;   }
    RTAC_HOSTDEVICE       float& bearing()         { return bearing_;   }
    RTAC_HOSTDEVICE const float& elevation() const { return elevation_; }
    RTAC_HOSTDEVICE       float& elevation()       { return elevation_; }

    RTAC_HOSTDEVICE static SimSample3D Make(const Complex<float>& value,
                                            float travel,
                                            const float3& dir)
    {
        SimSample3D res;
        res.value_     = value;
        res.travel_    = travel;
        res.bearing_   = atan2(dir.y, dir.x);
        res.elevation_ = atan2(dir.z, dir.x);
        return res;
    }

    RTAC_HOSTDEVICE static SimSample3D Null()
    {
        SimSample3D res{};
        res.travel_ = -1.0f;
        return res;
    }
};

/**
 * This is the preferred way to represent a simulation sample.
 *
 * For acoustic simulation, each contribution to the simulated sensor output is
 * represented by a complex response and a 2D location.
 *
 * The PointT type is expected to be a CUDA vector type such as float2/3/4. It
 * represents a 3D location in space expressed by default in cartesian
 * coordinates (but are used as polar coordinates in some conditions).
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

    RTAC_HOSTDEVICE Sample<T,P>& operator*=(T a) {
        datum *= a;
        return *this;
    }

    RTAC_HOSTDEVICE Sample<T,P>& operator*=(float2 a) {
        datum = Complex<T>{datum.real()*a.x - datum.imag()*a.y,
                           datum.real()*a.y + datum.imag()*a.x};
        return *this;
    }
};

// below are derived types for convenience
template <typename T>
struct Sample3D : public Sample<T, float3>
{
    using Scalar = T;
    using Point  = float3;

    RTAC_HOSTDEVICE float x() const { return this->position.x; }
    RTAC_HOSTDEVICE float y() const { return this->position.y; }
    RTAC_HOSTDEVICE float z() const { return this->position.z; }

    RTAC_HOSTDEVICE float& x() { return this->position.x; }
    RTAC_HOSTDEVICE float& y() { return this->position.y; }
    RTAC_HOSTDEVICE float& z() { return this->position.z; }

    RTAC_HOSTDEVICE static Sample3D<T> Make(const Complex<T>& value, const float3& p) {
        //return Sample3D<T>{value, p}; // for compatibility with cuda < 11 (c++14 max)
        Sample3D<T> res;
        res.datum    = value;
        res.position = p;
        return res;
    }
    RTAC_HOSTDEVICE static Sample3D<T> Zero() {
        return Sample3D<T>::Make({0,0}, {0,0,0});
    }
};

template <typename T>
struct PolarSample2D : public Sample<T, float2>
{
    using Scalar = T;
    using Point  = float2;

    RTAC_HOSTDEVICE float range() const { return this->position.x; }
    RTAC_HOSTDEVICE float theta() const { return this->position.y; }

    RTAC_HOSTDEVICE float& range() { return this->position.x; }
    RTAC_HOSTDEVICE float& theta() { return this->position.y; }
    
    // p is assumed in polar coordinates
    RTAC_HOSTDEVICE static PolarSample2D<T> Make(const Complex<T>& value, const float2& p) {
        //return PolarSample2D<T>{value, p}; // for compatibility with cuda < 11 (c++14 max)
        PolarSample2D<T> res;
        res.datum    = value;
        res.position = p;
        return res;
    }
    // p is assumed to be expressed in cartesian coordinates.
    RTAC_HOSTDEVICE static PolarSample2D<T> Make(const Complex<T>& value, const float3& p) {
        //return PolarSample2D<T>{value, float2{length(p), -atan2(p.y, p.x)}};
        PolarSample2D<T> res;
        res.datum    = value;
        res.position = float2{length(p), -atan2(p.y, p.x)};
        return res;
    }
    // other.position is assumed to be expressed in cartesian coordinates.
    RTAC_HOSTDEVICE static PolarSample2D<T> Make(const Sample3D<T>& other) {
        return PolarSample2D<T>::Make(other.datum(), other.position);
    }
    RTAC_HOSTDEVICE static PolarSample2D<T> Zero() {
        return PolarSample2D<T>::Make({0,0},float2{0,0});
    }
};

}; //namespace simulation
}; //namespace rtac

template <typename T, typename P, typename T2>
RTAC_HOSTDEVICE inline rtac::simulation::Sample<T,P> operator*(const rtac::simulation::Sample<T,P>& s, T2 a)
{
    rtac::simulation::Sample<T,P> res(s);
    res *= a;
    return res;
}
template <typename T, typename P, typename T2>
RTAC_HOSTDEVICE inline rtac::simulation::Sample<T,P> operator*(T2 a, const rtac::simulation::Sample<T,P>& s)
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
