//
// Created by lasagnaphil on 2018-09-17.
//

#ifndef FLUID_SIM_FLUIDSIMSETTINGS_H
#define FLUID_SIM_FLUIDSIMSETTINGS_H

struct FluidSimSettings {
    struct Dim3D {
        static constexpr int SIZEX = 64;
        static constexpr int SIZEY = 64;
        static constexpr int SIZEZ = 64;
        static constexpr bool ENABLE_DEBUG_UI = false;
    };

};

#endif //FLUID_SIM_FLUIDSIMSETTINGS_H
