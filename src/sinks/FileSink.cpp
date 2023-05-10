#include <rtac_simulation/sinks/FileSink.h>

#include <rtac_base/files.h>
#include <rtac_base/serialization/sonar_ping.h>

namespace rtac { namespace simulation {

FileSink::FileSink(const std::string& filename, const std::string& sinkName) :
    Sink(sinkName),
    filename_(filename),
    file_(filename, std::ofstream::binary)
{}

FileSink::Ptr FileSink::Create(const std::string& filename,
                               bool  overwrite,
                               const std::string& sinkName)
{
    if(!files::prepare_path(filename, overwrite)) {
        throw FileError() << " : could not prepare path for file '"
                          << filename  << '\'';
    }
    return Ptr(new FileSink(filename, sinkName));
}

void FileSink::set_output(const SensorInstance::Ptr& sensor)
{
    if(!sensor->is_type<SensorInstance2D>()) {
        std::cerr << "FileSink : sensor type not handled" << std::endl;
        return;
    }
    switch(sensor->scalar_type()) {
        default: throw TypeError() << " : only float and Complex<float> are supported";
        case RTAC_FLOAT:
            serialize(file_, 
                      sensor->safe_cast<SensorInstance2D_2<float>>()->get_ping());
            break;
        case RTAC_CFLOAT:
            serialize(file_, 
                      sensor->safe_cast<SensorInstance2D_2<Complex<float>>>()->get_ping());
            break;
    }
}

} //namespace simulation
} //namespace rtac

