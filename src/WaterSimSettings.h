//
// Created by lasagnaphil on 2018-09-17.
//

#ifndef FLUID_SIM_WATERSIMSETTINGS_H
#define FLUID_SIM_WATERSIMSETTINGS_H

struct WaterSimSettings {
    struct Dim3D {
        static constexpr int SIZEX = 64;
        static constexpr int SIZEY = 64;
        static constexpr int SIZEZ = 64;
        static constexpr bool ENABLE_DEBUG_UI = false;
    };

    struct Dim2D {
        static constexpr int SIZEX = 48;
        static constexpr int SIZEY = 48;
        static constexpr double DT = 0.0008;
        static constexpr double DX = 0.001;
        static constexpr int PARTICLES_PER_CELL_SQRT = 4;
    };
};

#endif //FLUID_SIM_WATERSIMSETTINGS_H
