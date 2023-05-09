#ifndef _DEF_RTAC_SIMULATION_SINK_2D_H_
#define _DEF_RTAC_SIMULATION_SINK_2D_H_

#include <memory>
#include <fstream>

#include <rtac_base/Exception.h>
#include <rtac_base/types/SonarPing.h>

#include <rtac_simulation/Sink.h>
#include <rtac_simulation/SensorInstance2D.h>

namespace rtac { namespace simulation {

class Sink2D : public Sink
{
    public:

    using Ptr      = std::shared_ptr<Sink2D>;
    using ConstPtr = std::shared_ptr<const Sink2D>;

    protected:

    Sink2D() {}

    public:

    virtual void set_output(const SensorInstance::Ptr& sensor) override;
    virtual void set_output(const cuda::CudaPing2D<float>& ping)          = 0; 
    virtual void set_output(const cuda::CudaPing2D<Complex<float>>& ping) = 0;
};

class FileSink2D : public Sink2D
{
    public:

    using Ptr      = std::shared_ptr<Sink2D>;
    using ConstPtr = std::shared_ptr<const Sink2D>;
    
    protected:

    std::string   filename_;
    std::ofstream file_;

    FileSink2D(const std::string& filename);

    public:

    static Ptr Create(const std::string& filename, bool overwrite = false);

    const std::string& filename() const { return filename_; }

    void set_output(const cuda::CudaPing2D<float>& ping)          override;
    void set_output(const cuda::CudaPing2D<Complex<float>>& ping) override;
};

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_SINK_2D_H_


