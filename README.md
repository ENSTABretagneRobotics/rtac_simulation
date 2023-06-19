# rtac_simulation

This is the main repository for the RTAC simulation framwork.

The RTAC simulator is an efficient sonar simulator which aims to be used inside
AUV.

It is base on the CUDA framework, the OptiX ray tracing library and OpenGL.

## Installation

This is a regular cmake package. Its dependencies are
[rtac_base](https://github.com/ENSTABretagneRobotics/rtac_base),
[rtac_display](https://github.com/ENSTABretagneRobotics/rtac_display), and
[rtac_optix](https://github.com/ENSTABretagneRobotics/rtac_optix).

## Configuration

You should persistently add this variable to your environment :

```
echo "export RTAC_CONFIG_PATH=<library_install_location>/share/rtac_simulation/examples" >> $HOME/.bashrc
```

This will give you access to some example configuration files. You may populate
the RTAC_CONFIG_PATH variable with your own configuration paths (; separated
directory list).

## Getting started

Documentation is sparse. The ray_caster03 example should run right away and may
be the base for your own developments.
