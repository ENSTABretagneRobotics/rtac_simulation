#include <rtac_simulation/sinks/Sink2D.h>

#include <rtac_base/files.h>
#include <rtac_base/serialization/sonar_ping.h>

namespace rtac { namespace simulation {

void Sink2D::set_output(const SensorInstance::Ptr& sensor)
{
    if(!sensor->is_type<SensorInstance2D>()) {
        std::cerr << "Wrong type for Sink2D" << std::endl;
        return;
    }
    switch(sensor->scalar_type()) {
        default: throw TypeError() << " : only float and Complex<float> are supported";
        case RTAC_FLOAT:
            this->set_output(sensor->safe_cast<SensorInstance2D_2<float>>()->get_ping());
            break;
        case RTAC_CFLOAT:
            this->set_output(sensor->safe_cast<SensorInstance2D_2<Complex<float>>>()->get_ping());
            break;
    }
}

FileSink2D::FileSink2D(const std::string& filename, const std::string& sinkName) :
    Sink2D(sinkName),
    filename_(filename),
    file_(filename, std::ofstream::binary)
{}

FileSink2D::Ptr FileSink2D::Create(const std::string& filename, bool overwrite,
                                   const std::string& sinkName)
{
    if(!files::prepare_path(filename, overwrite)) {
        throw FileError() << " : could not prepare path for file '"
                          << filename  << '\'';
    }
    return Ptr(new FileSink2D(filename, sinkName));
}

void FileSink2D::set_output(const cuda::CudaPing2D<float>& ping)
{
    serialize(file_, ping);
}

void FileSink2D::set_output(const cuda::CudaPing2D<Complex<float>>& ping)
{
    serialize(file_, ping);
}

} //namespace simulation
} //namespace rtac
