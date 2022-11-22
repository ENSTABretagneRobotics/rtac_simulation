#ifndef _DEF_RTAC_SIMULATION_TARGET_2D_H_
#define _DEF_RTAC_SIMULATION_TARGET_2D_H_

#include <memory>

#include <rtac_base/types/VectorView.h>
#include <rtac_base/types/Image.h>

#include <rtac_base/cuda_defines.h>
#include <rtac_base/cuda/DeviceVector.h>

namespace rtac { namespace simulation {

template <typename T>
struct TargetView2D
{
    public:

    using value_type = T;

    using Image = rtac::ImageView<T>;
    using Shape = typename Image::Shape;

    Image  data_;
    float* widthCoordinates_;
    float* heightCoordinates_;

    RTAC_HOSTDEVICE uint32_t width()  const { return data_.width();  }
    RTAC_HOSTDEVICE uint32_t height() const { return data_.height(); }
    RTAC_HOSTDEVICE Shape    shape()  const { return data_.shape();  }
    RTAC_HOSTDEVICE auto     size()   const { return data_.size();  }

    #ifdef RTAC_CUDACC

    __device__ T  operator()(uint32_t i, uint32_t j) const { return data_(i,j); }
    __device__ T& operator()(uint32_t i, uint32_t j)       { return data_(i,j); }

    __device__ float width_coordinate(uint32_t w)  const { return widthCoordinates_[w]; }
    __device__ float height_coordinate(uint32_t h) const { return heightCoordinates_[h]; }

    #endif //RTAC_CUDACC
};

template <typename T>
class Target2D
{
    public:

    using Ptr      = std::shared_ptr<Target2D<T>>;
    using ConstPtr = std::shared_ptr<const Target2D<T>>;
    
    using Image = rtac::Image<T, rtac::cuda::DeviceVector>;
    using Shape = typename Image::Shape;
    
    protected:

    Image data_; 
    cuda::DeviceVector<float> widthCoordinates_;
    cuda::DeviceVector<float> heightCoordinates_;

    Target2D() {}
    Target2D(const cuda::DeviceVector<float>& widthCoordinates,
             const cuda::DeviceVector<float>& heightCoordinates);
    
    public:

    static Ptr Create() { return Ptr(new Target2D<T>()); }
    static Ptr Create(const cuda::DeviceVector<float>& widthCoordinates,
                      const cuda::DeviceVector<float>& heightCoordinates);

    void update_data_size();
    void set_width_coordinates(const cuda::DeviceVector<float>& widthCoordinates);
    void set_height_coordinates(const cuda::DeviceVector<float>& heightCoordinates);
    void set_coordinates(const cuda::DeviceVector<float>& widthCoordinates,
                         const cuda::DeviceVector<float>& heightCoordinates);

    cuda::DeviceVector<float>& width_coordinates()  { return widthCoordinates_; }
    cuda::DeviceVector<float>& height_coordinates() { return heightCoordinates_; }
    const cuda::DeviceVector<float>& width_coordinates()  const { return widthCoordinates_; }
    const cuda::DeviceVector<float>& height_coordinates() const { return heightCoordinates_; }

    uint32_t width()  const { return data_.width();  }
    uint32_t height() const { return data_.height(); }
    Shape    shape()  const { return data_.shape();  }
    auto     size()   const { return data_.size();   }

    TargetView2D<T>       view();
    TargetView2D<const T> view() const;

    Image&       data()       { return data_; }
    const Image& data() const { return data_; }
};

template <typename T>
Target2D<T>::Target2D(const cuda::DeviceVector<float>& widthCoordinates,
                      const cuda::DeviceVector<float>& heightCoordinates) :
    Target2D<T>()
{
    this->set_coordinates(widthCoordinates, heightCoordinates);
}

template <typename T>
typename Target2D<T>::Ptr Target2D<T>::Create(const cuda::DeviceVector<float>& widthCoordinates,
                                              const cuda::DeviceVector<float>& heightCoordinates)
{
    return Ptr(new Target2D<T>(widthCoordinates, heightCoordinates));
}

template <typename T>
void Target2D<T>::update_data_size()
{
    if(data_.width()  != widthCoordinates_.size() ||
       data_.height() != heightCoordinates_.size()) {
        data_.resize({widthCoordinates_.size(),
                      heightCoordinates_.size()});
    }
}

template <typename T>
void Target2D<T>::set_width_coordinates(const cuda::DeviceVector<float>& widthCoordinates)
{
    widthCoordinates_  = widthCoordinates;
    this->update_data_size();
}

template <typename T>
void Target2D<T>::set_height_coordinates(const cuda::DeviceVector<float>& heightCoordinates)
{
    heightCoordinates_ = heightCoordinates;
    this->update_data_size();
}

template <typename T>
void Target2D<T>::set_coordinates(const cuda::DeviceVector<float>& widthCoordinates,
                                  const cuda::DeviceVector<float>& heightCoordinates)
{
    widthCoordinates_  = widthCoordinates;
    heightCoordinates_ = heightCoordinates;
    this->update_data_size();
}

template <typename T>
TargetView2D<T> Target2D<T>::view()
{ 
    return TargetView2D<T>{data_.view(),
                           widthCoordinates_.data(),
                           heightCoordinates_.data()};
}

template <typename T>
TargetView2D<const T> Target2D<T>::view() const
{ 
    return TargetView2D<const T>{data_.view(),
                                 widthCoordinates_.data(),
                                 heightCoordinates_.data()};
}

}; //namespace simulation
}; //namespace rtac

#endif //_DEF_RTAC_SIMULATION_TARGET_2D_H_
