#include <rtac_simulation/examples/blueprint_oculus.h>

#include <rtac_simulation/helpers/receiver_factories.h>

namespace rtac { namespace simulation {

OculusReceiver::Kernel::ConstPtr OculusReceiver::make_lf_kernel(float rangeResolution)
{
    float bearingResolution = 0.6f;
    float bearingSpan       = 130.0f;
    float pulseLength       = 2*rangeResolution;
    //float waveLength        = 0.0012178;
    float waveLength        = 1500.0 / 1.2e6;

    return simple_polar_kernel<float>(bearingResolution, bearingSpan,
                                      pulseLength, waveLength);
}

OculusReceiver::Kernel::ConstPtr OculusReceiver::make_hf_kernel(float rangeResolution)
{
    float bearingResolution = 0.4f;
    float bearingSpan       = 80.0f;
    float pulseLength       = 2*rangeResolution;
    float waveLength        = 1500.0 / 2.1e6;

    return simple_polar_kernel<float>(bearingResolution, bearingSpan,
                                      pulseLength, waveLength);
}

OculusReceiver::OculusReceiver() :
    PolarReceiver2D<float>(nullptr, nullptr, nullptr),
    currentMode_(ModeNone),
    lfDirectivity_(Directivity::from_sinc_parameters(130.0f, 20.0f)),
    hfDirectivity_(Directivity::from_sinc_parameters(80.0f, 12.0f))
{}

bool OculusReceiver::needs_update(FrequencyMode requestedMode, float maxRange,
                                  unsigned int nRanges, unsigned int nBearings)
{
    if(requestedMode != currentMode_) {
        return true;
    }
    if(requestedMode == LowFrequency) {
        if(!lfTarget_ || !lfKernel_
            || lfTarget_->needs_update(0.1f, maxRange, nRanges, nBearings))
        {
            return true;
        }
        if(this->directivity().get() != lfDirectivity_.get()
           || this->target().get()   != lfTarget_.get()
           || this->psf().get()      != lfKernel_.get())
        {
            return true;
        }
    }
    else if(requestedMode == HighFrequency) {
        if(!hfTarget_ || !hfKernel_
            || hfTarget_->needs_update(0.1f, maxRange, nRanges, nBearings))
        {
            return true;
        }
        if(this->directivity().get() != hfDirectivity_.get()
           || this->target().get()   != hfTarget_.get()
           || this->psf().get()      != hfKernel_.get())
        {
            return true;
        }
    }
    return false;
}

bool OculusReceiver::reconfigure(FrequencyMode mode,
                                 float maxRange, unsigned int nRanges,
                                 const std::vector<float>& bearings,
                                 bool forceReconfigure)
{
    std::cout << "reconfigure" << std::endl;
    if(mode == ModeNone) {
        std::cerr << "OculusReceiver : Invalid requested frequency mode. "
                  << "Ignoring reconfiguration." << std::endl;
        return false;
    }

    if(!this->needs_update(mode, maxRange, nRanges, bearings.size()) && !forceReconfigure)
        return false;

    std::vector<float> ranges(nRanges);
    for(int n = 0; n < nRanges; n++) {
        ranges[n] = ((maxRange - 0.1f) * n) / (nRanges - 1) + 0.1f;
    }
    if(mode == LowFrequency)
    {
        if(!lfTarget_) {
            lfTarget_ = PolarTarget2D<Complex<float>>::Create(bearings, ranges);
        }
        else {
            lfTarget_->update(bearings, ranges);
        }
        lfKernel_          = make_lf_kernel(lfTarget_->range_resolution());
        this->target_      = lfTarget_;
        this->psf_         = lfKernel_;
        this->directivity_ = lfDirectivity_;
        currentMode_       = LowFrequency;
        return true;
    }
    else if(mode == HighFrequency)
    {
        if(!hfTarget_) {
            hfTarget_ = PolarTarget2D<Complex<float>>::Create(bearings, ranges);
        }
        else {
            hfTarget_->update(bearings, ranges);
        }
        hfKernel_          = make_hf_kernel(hfTarget_->range_resolution());
        this->target_      = hfTarget_;
        this->psf_         = hfKernel_;
        this->directivity_ = hfDirectivity_;
        currentMode_       = HighFrequency;
    }
    // should never happen
    return false;
}

bool OculusReceiver::reconfigure(const OculusSimplePingResult& metadata,
                                 const uint8_t* data,
                                 bool forceReconfigure)
{
    FrequencyMode requestedMode = ModeNone;
    if(metadata.fireMessage.masterMode == 1) {
        requestedMode      = LowFrequency;
        this->target_      = lfTarget_;
        this->psf_         = lfKernel_;
        this->directivity_ = lfDirectivity_;
    }
    else if(metadata.fireMessage.masterMode == 2) {
        requestedMode      = HighFrequency;
        this->target_      = hfTarget_;
        this->psf_         = hfKernel_;
        this->directivity_ = hfDirectivity_;
    }
    else {
        std::cerr << "OculusReceiver : Invalid requested frequency mode : "
                  << (int)metadata.fireMessage.masterMode 
                  << ". Ignoring reconfiguration." << std::endl;
        return false;
    }

    if(forceReconfigure || this->needs_update(requestedMode, 
                                              metadata.fireMessage.range,
                                              metadata.nRanges,
                                              metadata.nBeams))
    {
        std::vector<float> bearingData(metadata.nBeams);
        auto bearings = (const int16_t*)(data + sizeof(OculusSimplePingResult));
        for(int i = 0; i < metadata.nBeams; i++) {
            bearingData[i] = (0.01f*M_PI/180.0f) * bearings[i];
        }
        return this->reconfigure(requestedMode, metadata.fireMessage.range,
                                 metadata.nRanges, bearingData, true);
    }

    return false;
}

bool OculusSensor::needs_update(FrequencyMode requestedMode, 
                                float minRange, float maxRange,
                                unsigned int nRanges, unsigned int nBearings)
{
    if(requestedMode != currentMode_) {
        return true;
    }

    auto sensor_needs_update = [&](const SensorModel::Ptr& sensor) {
        return !sensor 
            || sensor->data().width()  != nBearings
            || sensor->data().height() != nRanges
            || sensor->data().height_dim().bounds().lower != minRange
            || sensor->data().height_dim().bounds().upper != maxRange;
    };
    if(requestedMode == LowFrequency) {
        return sensor_needs_update(lfSensor_);
    }
    else if(requestedMode == HighFrequency) {
        return sensor_needs_update(hfSensor_);
    }
    return false;
}

Kernel2D<float> OculusSensor::make_lf_kernel(float rangeResolution)
{
    float bearingResolution   = 0.6f;
    float bearingSpan         = 130.0f;
    float pulseLength         = 2*rangeResolution;
    float waveLength          = 1500.0 / 1.2e6;
    float numPeriod           = pulseLength / waveLength;
    unsigned int oversampling = 8;

    signal::SincFunction<float> bearingPsf(bearingSpan / bearingResolution, oversampling);
    signal::SineFunction<float> rangePsf(numPeriod, oversampling);

    unsigned int W = bearingPsf.size();
    unsigned int H = rangePsf.size();

    std::cout << "kernel : " << W << 'x' << H << std::endl;
    std::cout << "pulse length : " << pulseLength << std::endl;

    Image<float> kernelData(W,H);
    for(int h = 0; h < H; h++) {
        for(int w = 0; w < W; w++) {
            //kernelData(h,w) = rangePsf.function()[h];
            //kernelData(h,w) = bearingPsf.function()[w];
            kernelData(h,w) = rangePsf.function()[h] * bearingPsf.function()[w];
            //kernelData(h,w) = 1.0f;
        }
    }
    
    return Kernel2D<float>(bearingSpan * M_PI / 180.0f, pulseLength, kernelData);
}

Kernel2D<float> OculusSensor::make_hf_kernel(float rangeResolution)
{
    float bearingResolution   = 0.4f;
    float bearingSpan         = 80.0f;
    float pulseLength         = 2*rangeResolution;
    float waveLength          = 1500.0 / 2.1e6;
    float numPeriod           = pulseLength / waveLength;
    unsigned int oversampling = 8;

    signal::SincFunction<float> bearingPsf(bearingSpan / bearingResolution, oversampling);
    signal::SineFunction<float> rangePsf(numPeriod, oversampling);

    unsigned int W = bearingPsf.size();
    unsigned int H = rangePsf.size();

    Image<float> kernelData(W,H);
    for(int h = 0; h < H; h++) {
        for(int w = 0; w < W; w++) {
            //kernelData(h,w) = rangePsf.function()[h];
            kernelData(h,w) = bearingPsf.function()[w];
            //kernelData(h,w) = rangePsf.function()[h] * bearingPsf.function()[w];
            //kernelData(h,w) = 1.0f;
        }
    }
    
    return Kernel2D<float>(bearingSpan * M_PI / 180.0f, pulseLength, kernelData);
}

bool OculusSensor::reconfigure(FrequencyMode mode,
                               float minRange, float maxRange, 
                               unsigned int nRanges,
                               const std::vector<float>& bearings,
                               bool forceReconfigure)
{
    std::cout << "reconfigure" << std::endl;
    if(mode == ModeNone) {
        std::cerr << "OculusReceiver : Invalid requested frequency mode. "
                  << "Ignoring reconfiguration." << std::endl;
        return false;
    }

    if(!forceReconfigure && !this->needs_update(mode, minRange, maxRange, nRanges, bearings.size()))
        return false;

    std::vector<float> ranges(nRanges);
    for(int n = 0; n < nRanges; n++) {
        ranges[n] = ((maxRange - minRange) * n) / (nRanges - 1) + minRange;
    }
    if(mode == LowFrequency)
    {
        if(!lfSensor_) {
            lfSensor_ = SensorModel::Create(bearings,
                                            LinearDim(nRanges,  {minRange, maxRange}),
                                            make_lf_kernel((maxRange - minRange) / (nRanges - 1)));
        }
        else {
            lfSensor_->reconfigure(bearings, LinearDim(nRanges, {minRange, maxRange}));
            lfSensor_->point_spread_function() = 
                make_lf_kernel((maxRange - minRange) / (nRanges - 1));
        }
        currentMode_ = LowFrequency;
        return true;
    }
    else if(mode == HighFrequency)
    {
        if(!hfSensor_) {
            hfSensor_ = SensorModel::Create(bearings,
                                            LinearDim(nRanges,  {minRange, maxRange}),
                                            make_hf_kernel((maxRange - minRange) / (nRanges - 1)));
        }
        else {
            hfSensor_->reconfigure(bearings, LinearDim(nRanges, {minRange, maxRange}));
            hfSensor_->point_spread_function() = 
                make_hf_kernel((maxRange - minRange) / (nRanges - 1));
        }
        currentMode_ = HighFrequency;
        return true;
    }
    // should never happen
    return false;
}

bool OculusSensor::reconfigure(const OculusSimplePingResult& metadata,
                               const uint8_t* data,
                               bool forceReconfigure)
{
    FrequencyMode requestedMode = ModeNone;
    if(metadata.fireMessage.masterMode == 1) {
        requestedMode      = LowFrequency;
    }
    else if(metadata.fireMessage.masterMode == 2) {
        requestedMode      = HighFrequency;
    }
    else {
        std::cerr << "OculusReceiver : Invalid requested frequency mode : "
                  << (int)metadata.fireMessage.masterMode 
                  << ". Ignoring reconfiguration." << std::endl;
        return false;
    }

    if(forceReconfigure || this->needs_update(requestedMode, 0.1f,
                                              metadata.fireMessage.range,
                                              metadata.nRanges,
                                              metadata.nBeams))
    {
        std::vector<float> bearingData(metadata.nBeams);
        auto bearings = (const int16_t*)(data + sizeof(OculusSimplePingResult));
        for(int i = 0; i < metadata.nBeams; i++) {
            bearingData[i] = (0.01f*M_PI/180.0f) * bearings[i];
            //bearingData[i] = (0.01f*M_PI/180.0f) * bearings[metadata.nBeams - 1 - i];
        }
        return this->reconfigure(requestedMode, 0.1f, metadata.fireMessage.range,
                                 metadata.nRanges, bearingData, true);
    }

    return false;
}


} //namespace simulation
} //namespace rtac

