#ifndef _DEF_RTAC_SIMULATION_HELPER_OCULUS_RENDERER_H_
#define _DEF_RTAC_SIMULATION_HELPER_OCULUS_RENDERER_H_

#include <memory>

#include <rtac_base/containers/Image.h>

#include <rtac_display/renderers/FanRenderer.h>

#include <narval_oculus/Oculus.h>

namespace rtac { namespace display {

inline Image<float> build_ping_image(const OculusSimplePingResult& metadata, const uint8_t* data)
{
    Image<float> pingData(metadata.nBeams, metadata.nRanges);

    if(metadata.fireMessage.flags | 0x4) {
        auto raw = data + metadata.imageOffset;
        for(int r = 0; r < metadata.nRanges; r++) {
            float gain = 1.0f / sqrt((float)*((const uint32_t*)raw));
            raw += 4;
            for(int b = 0; b < metadata.nBeams; b++) {
                pingData(r,b) = gain * raw[b];
            }
            raw += metadata.nBeams;
        }
    }
    else {
        auto raw = data + metadata.imageOffset;
        auto ptr = pingData.data();
        for(int i = 0; i < pingData.size(); i++) {
            ptr[i] = raw[i] / 255.0;
        }
    }
    return pingData;
}

class OculusRenderer : public FanRenderer
{
    public:

    using Ptr      = std::shared_ptr<OculusRenderer>;
    using ConstPtr = std::shared_ptr<const OculusRenderer>;

    protected:

    uint8_t currentMasterMode_;
    GLVector<float> pingData_;

    OculusRenderer(const GLContext::Ptr& context) :
        FanRenderer(context),
        currentMasterMode_(255)
    {
        this->set_direction(FanRenderer::Direction::Down);
    }

    public:

    static Ptr Create(const GLContext::Ptr& context) {
        return Ptr(new OculusRenderer(context));
    }

    void set_data(const OculusSimplePingResult& metadata, const uint8_t* data)
    {
        this->update_data(metadata, data);
        this->set_range({0.0f, (float)metadata.fireMessage.range});

        if(metadata.fireMessage.masterMode == currentMasterMode_) {
            return;
        }

        std::vector<float> bearings(metadata.nBeams);
        auto bearingData = (const int16_t*)(data + sizeof(OculusSimplePingResult));
        for(int i = 0; i < bearings.size(); i++) {
            bearings[i] = (0.01*M_PI / 180.0) * bearingData[i];
        }
        this->set_bearings(bearings.size(), bearings.data());
    }

    void update_data(const OculusSimplePingResult& metadata, const uint8_t* data)
    {
        pingData_.resize(metadata.nBeams*metadata.nRanges);

        if(metadata.fireMessage.flags | 0x4) {
            auto ptr = pingData_.map();
            auto raw = data + metadata.imageOffset;
            for(int r = 0; r < metadata.nRanges; r++) {
                float gain = 1.0f / sqrt((float)*((const uint32_t*)raw));
                raw += 4;
                for(int b = 0; b < metadata.nBeams; b++) {
                    ptr[metadata.nBeams*r + b] = gain * raw[b];
                }
                raw += metadata.nBeams;
            }
        }
        else {
            auto raw = data + metadata.imageOffset;
            auto ptr = pingData_.map();
            for(int i = 0; i < pingData_.size(); i++) {
                ptr[i] = raw[i] / 255.0;
            }
        }
        this->FanRenderer::set_data({metadata.nBeams, metadata.nRanges}, pingData_);
    }
};

} //namespace display
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_HELPER_OCULUS_RENDERER_H_
