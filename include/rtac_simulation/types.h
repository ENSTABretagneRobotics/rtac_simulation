#ifndef _DEF_RTAC_SIMULATION_TYPES_H_
#define _DEF_RTAC_SIMULATION_TYPES_H_

#include <rtac_base/types/VectorView.h>
#include <rtac_base/types/Image.h>
#include <rtac_base/types/MappedGrid.h>
#include <rtac_base/types/GridMap.h>

namespace rtac { namespace simulation {

/**
 * This is the default container for 2D simulation output. It simply defines an
 * 2D array in which pixels can be accessed/modified using indexes (ex :
 * img(row,col) = 1.0). Data is store in row major format.
 *
 * T is the pixel output type. Usually a single float but can be a
 * cuda::Complex<float>.
 *
 * The container type is a VectorView which contains a raw pointer to cuda
 * device memory. The data must be managed elsewhere.
 */
template <typename T>
using TargetData2D = types::Image<T, types::VectorView>;

/**
 * This defines a 2D output target for the simulation (for example for a front
 * scan sonar).
 *
 * MapT0 and MapT1 define mapping from the image index space (row,col) to a
 * physical dimension, such as (bearing,range).
 *
 * These mapping can be non-linear (for example the BluePrint Oculus sonar ping
 * polar image is not linearly sampled in the bearing dimension). MapT0 and
 * MapT1 must represent such non-uniform sampling.
 */
template <typename T, typename MapT0, typename MapT1>
using Target2D = types::MappedGrid<T, TargetData2D, MapT0, MapT1>;

}; //namespace simulation
}; //namespace rtac

#endif //_DEF_RTAC_SIMULATION_TYPES_H_


