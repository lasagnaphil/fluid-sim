//
// Created by lasagnaphil on 2018-09-17.
//

#ifndef FLUID_SIM_WATERSIMSETTINGS_H
#define FLUID_SIM_WATERSIMSETTINGS_H

struct WaterSimSettings {
    enum class SimMode {
        SemiLagrangian, PIC
    };
    struct Dim3D {
        static constexpr int SIZEX = 64;
        static constexpr int SIZEY = 64;
        static constexpr int SIZEZ = 64;
        static constexpr bool ENABLE_DEBUG_UI = false;
    };

    struct Dim2D {
        static constexpr int SIZEX = 128;
        static constexpr int SIZEY = 128;
        static constexpr double DT = 0.0005;
        static constexpr double DX = 0.001;
        static constexpr int PARTICLES_PER_CELL_SQRT = 2;
        static constexpr SimMode SIM_MODE = SimMode::PIC;
    };
};

#endif //FLUID_SIM_WATERSIMSETTINGS_H
