#ifndef _DEF_RTAC_SIMULATION_FILE_SINK_H_
#define _DEF_RTAC_SIMULATION_FILE_SINK_H_

#include <memory>
#include <fstream>

#include <rtac_base/Exception.h>
#include <rtac_base/types/SonarPing.h>

#include <rtac_simulation/SensorInstance2D.h>
#include <rtac_simulation/sinks/Sink.h>

namespace rtac { namespace simulation {

class FileSink : public Sink
{
    public:

    using Ptr      = std::shared_ptr<FileSink>;
    using ConstPtr = std::shared_ptr<const FileSink>;
    
    protected:

    std::string   filename_;
    std::ofstream file_;

    FileSink(const std::string& filename,
             const std::string& sinkName = "file-sink");

    public:

    static Ptr Create(const std::string& filename, bool overwrite = false,
                      const std::string& sinkName = "file-sink");

    const std::string& filename() const { return filename_; }

    virtual void set_output(const SensorInstance::Ptr& sensor) override;
};

} //namespace simulation
} //namespace rtac

#endif //_DEF_RTAC_SIMULATION_FILE_SINK_H_
