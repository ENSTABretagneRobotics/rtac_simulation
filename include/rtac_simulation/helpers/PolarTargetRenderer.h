#ifndef _DEF_RTAC_SIMULATION_HELPERS_POLAR_TARGET_RENDERERER_H_
#define _DEF_RTAC_SIMULATION_HELPERS_POLAR_TARGET_RENDERERER_H_

#include <rtac_base/types/Complex.h>
#include <rtac_base/cuda/utils.h>

#include <rtac_display/GLVector.h>
#include <rtac_display/renderers/FanRenderer.h>

#include <rtac_simulation/PolarTarget2D.h>

namespace rtac { namespace display {

class PolarTargetRenderer : public FanRenderer
{
    public:

    using Ptr      = std::shared_ptr<PolarTargetRenderer>;
    using ConstPtr = std::shared_ptr<const PolarTargetRenderer>;

    using Shape = FanRenderer::Shape;

    protected:

    GLVector<float> tmpData_;

    PolarTargetRenderer(const GLContext::Ptr& context);
    
    template <typename T>
    void set_geometry(typename rtac::simulation::PolarTarget2D<T>::ConstPtr data);

    public:

    static Ptr Create(const GLContext::Ptr& context) {
        return Ptr(new PolarTargetRenderer(context));
    }
    
    void set_data(rtac::simulation::PolarTarget2D<float>::ConstPtr data);
    void set_data(rtac::simulation::PolarTarget2D<Complex<float>>::ConstPtr data);
};

template <typename T>
void PolarTargetRenderer::set_geometry(typename rtac::simulation::PolarTarget2D<T>::ConstPtr data)
{
    HostVector<float> bearings = data->bearings();
    this->set_bearings(bearings.size(), bearings.data());
    this->set_range({data->range_min(), data->range_max()});
}

} //namespace display
} //namespace rtac


#endif //_DEF_RTAC_SIMULATION_HELPERS_POLAR_TARGET_RENDERERER_H_
