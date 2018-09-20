//
// Created by lasagnaphil on 2018-09-17.
//

#ifndef FLUID_SIM_WATERSIMSETTINGS_H
#define FLUID_SIM_WATERSIMSETTINGS_H

struct WaterSimSettings {
    struct Dim3D {
        static constexpr int SIZEX = 15;
        static constexpr int SIZEY = 15;
        static constexpr int SIZEZ = 15;
    };

    struct Dim2D {
        static constexpr int SIZEX = 100;
        static constexpr int SIZEY = 100;
    };
};

#endif //FLUID_SIM_WATERSIMSETTINGS_H
