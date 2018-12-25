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
        static constexpr double DT = 0.003;
        static constexpr double DX = 0.001;
        static constexpr double DR = DX * 0.9;
        static constexpr int PARTICLES_PER_CELL_SQRT = 2;
        static constexpr SimMode SIM_MODE = SimMode::SemiLagrangian;
        static constexpr int INIT_STATE = 2;
        static constexpr bool OSCILLATE_GRAVITY = true;
        static constexpr float OSCILLATE_GRAVITY_AMP = 3.0f;
        static constexpr float OSCILLATE_GRAVITY_PERIOD = 0.3f;
    }SemiLagrangian;
};

#endif //FLUID_SIM_WATERSIMSETTINGS_H
