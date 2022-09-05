#ifndef _DEF_RTAC_SIMULATION_POSE_3D_H_
#define _DEF_RTAC_SIMULATION_POSE_3D_H_

#include <iostream>
#include <iomanip>
#include <array>

#include <rtac_base/types/Pose.h>
#include <rtac_base/geometry.h>
#include <rtac_base/cuda_defines.h>
#include <rtac_base/cuda/utils.h>
#include <rtac_base/cuda/vec_math.h>

namespace rtac { namespace simulation {

/**
 * Column-major encoded 3x3 matrix (for ease of use in CUDA device code).
 */
template <typename T>
struct Matrix3 {

    std::array<T,9> data_;

    static Matrix3<T> Identity() {
        return Matrix3<T>{1,0,0,
                          0,1,0,
                          0,0,1};
    }

    T*       data()       { return data_.data(); }
    const T* data() const { return data_.data(); }
    
    // This should enable compile-time computation of linear array index from
    // pair of matrix indexes.
    RTAC_HOSTDEVICE static constexpr uint32_t linear_index(uint32_t row, uint32_t col) {
        return row + 3*col;
    }
    RTAC_HOSTDEVICE T& operator()(uint32_t row, uint32_t col) { 
        return data_[linear_index(row, col)];
    }
    RTAC_HOSTDEVICE T  operator()(uint32_t row, uint32_t col) const { 
        return data_[linear_index(row, col)];
    }

    RTAC_HOSTDEVICE Matrix3<T>& operator*=(const Matrix3<T>& rhs) {
        T tmp0, tmp1; // only two temp needed for processing;

        tmp0 = (*this)(0,0); tmp1 = (*this)(0,1);
        (*this)(0,0) = tmp0*rhs(0,0) + tmp1*rhs(1,0) + (*this)(0,2)*rhs(2,0);
        (*this)(0,1) = tmp0*rhs(0,1) + tmp1*rhs(1,1) + (*this)(0,2)*rhs(2,1);
        (*this)(0,2) = tmp0*rhs(0,2) + tmp1*rhs(1,2) + (*this)(0,2)*rhs(2,2);

        tmp0 = (*this)(1,0); tmp1 = (*this)(1,1);
        (*this)(1,0) = tmp0*rhs(0,0) + tmp1*rhs(1,0) + (*this)(1,2)*rhs(2,0);
        (*this)(1,1) = tmp0*rhs(0,1) + tmp1*rhs(1,1) + (*this)(1,2)*rhs(2,1);
        (*this)(1,2) = tmp0*rhs(0,2) + tmp1*rhs(1,2) + (*this)(1,2)*rhs(2,2);

        tmp0 = (*this)(2,0); tmp1 = (*this)(2,1);
        (*this)(2,0) = tmp0*rhs(0,0) + tmp1*rhs(1,0) + (*this)(2,2)*rhs(2,0);
        (*this)(2,1) = tmp0*rhs(0,1) + tmp1*rhs(1,1) + (*this)(2,2)*rhs(2,1);
        (*this)(2,2) = tmp0*rhs(0,2) + tmp1*rhs(1,2) + (*this)(2,2)*rhs(2,2);

        return *this;
    }
    
    RTAC_HOSTDEVICE Matrix3<T>& transpose() {
        std::swap((*this)(1,0), (*this)(0,1));
        std::swap((*this)(2,0), (*this)(0,2));
        std::swap((*this)(2,1), (*this)(1,2));
        return *this;
    }
    
    RTAC_HOSTDEVICE Matrix3<T> transposed() const {
        return Matrix3<T>{(*this)(0,0), (*this)(0,1), (*this)(0,2),
                          (*this)(1,0), (*this)(1,1), (*this)(1,2),
                          (*this)(2,0), (*this)(2,1), (*this)(2,2)};
    }

    Matrix3<T>& operator=(const Eigen::Matrix3<T>& mat) {
        Eigen::Map<Eigen::Matrix3<T>>(this->data()) = mat;
        return *this;
    }
};

/**
 * This is a small type to ease frame changes for vectors on device side.
 *
 * The rotation matrix in encoded in column-major order to be consistent with
 * the Eigen library (it is also consistent with OpenGL)
 *
 * This type intentionally does not use CUDA specific types to be compatible
 * with a pure C++ file compiled with a regular C++ compiler.
 *
 * R_ and T_ attributes are the rotation matrix and translation vector you
 * would find in an homogeneous 4x4 transformation matrix.
 */
template <typename T>
struct DevicePose {

    Matrix3<T> R_;
    float3 T_;

    static RTAC_HOSTDEVICE DevicePose Identity() { return DevicePose<T>{Matrix3<T>::Identity(), {0,0,0}}; }

    RTAC_HOSTDEVICE Matrix3<T>  rotation_matrix() const { return R_; }
    RTAC_HOSTDEVICE Matrix3<T>& rotation_matrix()       { return R_; }
    RTAC_HOSTDEVICE float3      translation() const     { return T_; }
    RTAC_HOSTDEVICE float3&     translation()           { return T_; }

    RTAC_HOSTDEVICE DevicePose<T>& operator*=(const DevicePose<T>& rhs);
    RTAC_HOSTDEVICE DevicePose<T> inverse() const;

    RTAC_HOSTDEVICE float3 to_world_frame(const float3& v) const;
    RTAC_HOSTDEVICE float3 to_local_frame(const float3& v) const;
    
    // This finds an othonormal matrix close to R_
    void sanities() {
        Eigen::Matrix3<T> tmp = Eigen::Map<Eigen::Matrix3<T>>(R_.data());
        R_ = rtac::geometry::orthonormalized(tmp); 
    }

    DevicePose<T>& operator=(const rtac::types::Pose<T>& other) {
        this->rotation_matrix() = other.rotation_matrix();
        T_.x = other.x();
        T_.y = other.y();
        T_.z = other.z();
        return *this;
    }
};

template <typename T> RTAC_HOSTDEVICE
inline DevicePose<T>& DevicePose<T>::operator*=(const DevicePose<T>& rhs)
{
    this->translation()     += this->rotation_matrix()*rhs.translation();
    this->rotation_matrix() *= rhs.rotation_matrix();
    return *this;
}

template <typename T> RTAC_HOSTDEVICE
inline DevicePose<T> DevicePose<T>::inverse() const
{
    // Rotation matrix inversion is transposition
    DevicePose<T> res;
    res.rotation_matrix() = this->rotation_matrix().transpose();
    res.translation() = -(res.rotation_matrix() * this->translation());
    return res;
}

template <typename T> RTAC_HOSTDEVICE 
inline float3 DevicePose<T>::to_world_frame(const float3& rhs) const
{
    return this->rotation_matrix()*rhs + this->translation();
}

template <typename T> RTAC_HOSTDEVICE
inline float3 DevicePose<T>::to_local_frame(const float3& rhs) const
{
    return this->rotation_matrix().transposed()*(rhs - this->translation());
}

} //namespace simulation
} //namespace rtac

template <typename T> RTAC_HOSTDEVICE
inline float3 operator*(const rtac::simulation::Matrix3<T>& A, const float3& v)
{
    return float3{A(0,0)*v.x + A(0,1)*v.y + A(0,2)*v.z,
                  A(1,0)*v.x + A(1,1)*v.y + A(1,2)*v.z,
                  A(2,0)*v.x + A(2,1)*v.y + A(2,2)*v.z};
}

template <typename T> RTAC_HOSTDEVICE
inline float3 operator*(const float3& v, const rtac::simulation::Matrix3<T>& A)
{
    return float3{v.x*A(0,0) + v.y*A(1,0) + v.z*A(2,0),
                  v.x*A(0,1) + v.y*A(1,1) + v.z*A(2,1),
                  v.x*A(0,2) + v.y*A(1,2) + v.z*A(2,2)};
}

template <typename T> RTAC_HOSTDEVICE
inline rtac::simulation::Matrix3<T> operator*(const rtac::simulation::Matrix3<T>& A,
                                              const rtac::simulation::Matrix3<T>& B)
{
    rtac::simulation::Matrix3<T> C;

    C(0,0) = A(0,0)*B(0,0) + A(0,0)*B(0,1) + A(0,0)*B(0,2);
    C(0,1) = A(0,1)*B(1,0) + A(0,1)*B(1,1) + A(0,1)*B(1,2);
    C(0,2) = A(0,2)*B(2,0) + A(0,2)*B(2,1) + A(0,2)*B(2,2);

    C(1,0) = A(1,0)*B(0,0) + A(1,0)*B(0,1) + A(1,0)*B(0,2);
    C(1,1) = A(1,1)*B(1,0) + A(1,1)*B(1,1) + A(1,1)*B(1,2);
    C(1,2) = A(1,2)*B(2,0) + A(1,2)*B(2,1) + A(1,2)*B(2,2);

    C(2,0) = A(2,0)*B(0,0) + A(2,0)*B(0,1) + A(2,0)*B(0,2);
    C(2,1) = A(2,1)*B(1,0) + A(2,1)*B(1,1) + A(2,1)*B(1,2);
    C(2,2) = A(2,2)*B(2,0) + A(2,2)*B(2,1) + A(2,2)*B(2,2);

    return C;
}

template <typename T> RTAC_HOSTDEVICE
inline float3 operator*(const rtac::simulation::DevicePose<T>& pose, const float3& v)
{
    return pose.to_world_frame(v);
}

template <typename T> RTAC_HOSTDEVICE
inline rtac::simulation::DevicePose<T> operator*(const rtac::simulation::DevicePose<T>& A,
                                                 const rtac::simulation::DevicePose<T>& B)
{
    return rtac::simulation::DevicePose<T>(A) *= B;
}

template <typename T>
inline std::ostream& operator<<(std::ostream& os, const rtac::simulation::Matrix3<T>& A)
{
    os << A(0,0) << " " << A(0,1) << " " << A(0,2) << "\n"
       << A(1,0) << " " << A(1,1) << " " << A(1,2) << "\n"
       << A(2,0) << " " << A(2,1) << " " << A(2,2) << "\n";
    return os;
}

template <typename T>
inline std::ostream& operator<<(std::ostream& os, const rtac::simulation::DevicePose<T>& pose)
{
    static constexpr int w = 8, p = 2;
    std::ostringstream oss;
    oss << std::setfill(' ') << std::setprecision(p);
    oss << std::setw(w) << pose.R_(0,0) << " " 
        << std::setw(w) << pose.R_(0,1) << " " 
        << std::setw(w) << pose.R_(0,2) << " | "
        << std::setw(w) << pose.T_.x    << "\n";
    oss << std::setw(w) << pose.R_(1,0) << " " 
        << std::setw(w) << pose.R_(1,1) << " " 
        << std::setw(w) << pose.R_(1,2) << " | "
        << std::setw(w) << pose.T_.y    << "\n";
    oss << std::setw(w) << pose.R_(2,0) << " " 
        << std::setw(w) << pose.R_(2,1) << " " 
        << std::setw(w) << pose.R_(2,2) << " | "
        << std::setw(w) << pose.T_.z    << "\n";
    os << oss.str();
    return os;
}

#endif //_DEF_RTAC_SIMULATION_POSE_3D_H_
