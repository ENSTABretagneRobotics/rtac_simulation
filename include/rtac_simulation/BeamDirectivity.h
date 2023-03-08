#ifndef _DEF_RTAC_SIMULATION_BEAM_DIRECTIVITY_H_
#define _DEF_RTAC_SIMULATION_BEAM_DIRECTIVITY_H_

#include <memory>
#include <iostream>

#include <rtac_base/types/Complex.h>
#include <rtac_base/containers/HostVector.h>
#include <rtac_base/cuda/DeviceVector.h>

namespace rtac { namespace simulation {

class BeamDirectivity;

/**
 * Iterator on BeamDirectivity value (no no-const iterator available).
 */
class BeamDirectivityIterator
{
    protected:

    const BeamDirectivity* beam_;
    unsigned int           index_;
    
    public:
    
    BeamDirectivityIterator() = default;
    BeamDirectivityIterator& operator=(const BeamDirectivityIterator&) = default;

    BeamDirectivityIterator(const BeamDirectivity* beam, unsigned int index) :
        beam_(beam),
        index_(index)
    {}

    BeamDirectivityIterator& operator++() { index_++; return *this; }
    BeamDirectivityIterator  operator++(int) const {
        BeamDirectivityIterator res(*this);
        (*this)++;
        return res;
    }

    bool operator==(const BeamDirectivityIterator& other) const {
        return beam_ == other.beam_ && index_ == other.index_;
    }
    bool operator!=(const BeamDirectivityIterator& other) const {
        return !(*this == other);
    }

    float operator*() const;
};

class BeamDirectivity
{
    public:

    using Ptr      = std::shared_ptr<BeamDirectivity>;
    using ConstPtr = std::shared_ptr<const BeamDirectivity>;

    protected:

    BeamDirectivity() = default;

    public:

    BeamDirectivityIterator begin() const { return BeamDirectivityIterator(this, 0);            }
    BeamDirectivityIterator end()   const { return BeamDirectivityIterator(this, this->size()); }

    virtual float        span() const = 0;
    virtual unsigned int size() const = 0;
    virtual float operator[](unsigned int idx) const = 0;
};

inline float BeamDirectivityIterator::operator*() const { return (*beam_)[index_]; }

class BeamDirectivity_Sinc : public BeamDirectivity
{
    public:

    using Ptr      = std::shared_ptr<BeamDirectivity>;
    using ConstPtr = std::shared_ptr<const BeamDirectivity>;

    protected:

    float span_;
    std::vector<float> data_;

    BeamDirectivity_Sinc(float span, const std::vector<float>& data) :
        span_(span),
        data_(data)
    {}

    public:

    static Ptr Create(float span, float angularResolution,
                      unsigned int oversampling = 8)
    {
        signal::SincFunction<float> sinc(span / angularResolution, oversampling);
        return Ptr(new BeamDirectivity_Sinc(span, sinc.function()));
    }

    float        span() const { return span_;        }
    unsigned int size() const { return data_.size(); }
    float operator[](unsigned int idx) const { return data_[idx]; }
};

} //namespace simulation
} //namespace rtac

inline std::ostream& operator<<(std::ostream& os, const rtac::simulation::BeamDirectivity& beam)
{
    os << "BeamDirectivity (span : " << beam.span()
       << ", size : " << beam.size() << ") :";
    for(auto v : beam) {
        os << ' ' << v;
    }
    return os;
}


#endif //_DEF_RTAC_SIMULATION_BEAM_DIRECTIVITY_H_
