# Eulerian Fluid Simulation

This is a (work in progress) fluid simulation library based on the methods in the book *Fluid Simulation for Computer Graphics* by Robert Bridson (2nd edition). 

It implements the following methods:

- Semi-Lagrangian Method
- PIC/FLIP Method (Interpolate between PIC and FLIP)

Only 2D version is implemented as of now. (I want to make sure the 2D version works perfectly before moving on to 3D.)

Features that really need to be implemented as of now:

- Particle reseeding
- Volume correction

Right now, the fluid simulation doesn't conserve volume very well, so these two will help in remedying this.

Features that might be implemented later on (if I have the time):

- Fluid rendering via Marching Cube method
- APIC (Affine Particle-in-Cell) method
- Support for solids not fixed on the grid
- Support for moving solids
- 3D simulation

# Requirements

- Compiler with C++14 support
- CPU with AVX/FMA support (As of now this is mandatory, because I've used some AVX/FMA intrinsics in code. Also, the math library I am using requires this too. Later on I would loosen up the requirements.)

These are the platforms I've tested on:

- Windows (MinGW, clang)
- Linux (Arch Linux, gcc or clang)

Note that if you're on Windows MinGW, you shouldn't use GCC because it has bugs with AVX/AVX2 instruction generation! (For a more detailed description, check the following issue: https://github.com/Alexpux/MSYS2-packages/issues/1209).

Optionally, you can enable OpenMP via the ``USE_OPENMP`` CMake option. (However, it doesn't work on MinGW clang for some unknown reasons)

# Running the demo

Linux (GCC or Clang)

```sh
git clone --recursive https://github.com/lasagnaphil/fluid-sim
cd fluid-sim
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make fluid_sim_demo -j$(nproc)
demo/fluid_sim_demo
```

Windows (in MinGW-w64 shell)
```sh
git clone --recursive https://github.com/lasagnaphil/fluid-sim
cd fluid-sim
mkdir build && cd build
cmake .. -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release
mingw32-make fluid_sim_demo
demo/fluid_sim_demo.exe
```

# Dependencies

For the library:

- [altlib](https://github.com/lasagnaphil/altlib): Alternative C++ Library
- [altmath](https://github.com/lasagnaphil/altmath): Alternative Math Library

These two are in the source code as git submodules. Make sure you clone them too! (using the ``--recursive`` flag)

For the demo:

- [glad](https://github.com/Dav1dde/glad): OpenGL function loader generator
- [imgui](https://github.com/ocornut/imgui): Dear ImGui
- [stb_image](https://github.com/nothings/stb): Image loading

# License

MIT License

Copyright (c) 2019 Philsik Chang

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
