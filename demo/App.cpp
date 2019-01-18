//
// Created by lasagnaphil on 9/9/18.
//

#include "App.h"

#include <glad/glad.h>
#include <imgui.h>
#include <imgui_impl_sdl.h>
#include <imgui_impl_opengl3.h>
#include "Defer.h"
#include "log.h"
#include <vec3.h>

#include "InputManager.h"

#include "FluidSim2D.h"
#include "FluidRenderer2D.h"

static void sdl_die(const char* message) {
    log_error("%s: %s\n", message, SDL_GetError());
    exit(1);
}

static void gl_debug_output(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar *message,
                     const void *userParam) {
    // ignore non-significant error/warning codes
    if(id == 131169 || id == 131185 || id == 131218 || id == 131204) return;

    log_debug("OpenGL: %s", message);

    switch (source)
    {
        case GL_DEBUG_SOURCE_API:             printf("Source: API"); break;
        case GL_DEBUG_SOURCE_WINDOW_SYSTEM:   printf("Source: Window System"); break;
        case GL_DEBUG_SOURCE_SHADER_COMPILER: printf("Source: Shader Compiler"); break;
        case GL_DEBUG_SOURCE_THIRD_PARTY:     printf("Source: Third Party"); break;
        case GL_DEBUG_SOURCE_APPLICATION:     printf("Source: Application"); break;
        case GL_DEBUG_SOURCE_OTHER:           printf("Source: Other"); break;
    }

    switch (type)
    {
        case GL_DEBUG_TYPE_ERROR:               printf("Type: Error"); break;
        case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR: printf("Type: Deprecated Behaviour"); break;
        case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR:  printf("Type: Undefined Behaviour"); break;
        case GL_DEBUG_TYPE_PORTABILITY:         printf("Type: Portability"); break;
        case GL_DEBUG_TYPE_PERFORMANCE:         printf("Type: Performance"); break;
        case GL_DEBUG_TYPE_MARKER:              printf("Type: Marker"); break;
        case GL_DEBUG_TYPE_PUSH_GROUP:          printf("Type: Push Group"); break;
        case GL_DEBUG_TYPE_POP_GROUP:           printf("Type: Pop Group"); break;
        case GL_DEBUG_TYPE_OTHER:               printf("Type: Other"); break;
    }

    switch (severity)
    {
        case GL_DEBUG_SEVERITY_HIGH:         printf("Severity: high"); break;
        case GL_DEBUG_SEVERITY_MEDIUM:       printf("Severity: medium"); break;
        case GL_DEBUG_SEVERITY_LOW:          printf("Severity: low"); break;
        case GL_DEBUG_SEVERITY_NOTIFICATION: printf("Severity: notification"); break;
    }
}

void App::init(vec2i screenSize) {
    settings.screenSize = screenSize;
    if (SDL_Init(SDL_INIT_VIDEO)) {
        sdl_die("Couldn't initialize SDL");
    }
    atexit(SDL_Quit);
    SDL_GL_LoadLibrary(NULL);

    // Use OpenGL Version 4.3
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_DEBUG_FLAG);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);

    // Create the window
    window = SDL_CreateWindow(
            "Water Simulation",
            SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
            screenSize.x, screenSize.y, SDL_WINDOW_OPENGL
    );

    if (window == NULL) sdl_die("Couldn't set video mode");

    // Create OpenGL Context
    glContext = SDL_GL_CreateContext(window);
    if (glContext == NULL)
        sdl_die("Failed to create OpenGL context");

    quit = false;

    // Check OpenGL properties
    log_info("OpenGL loaded");
    gladLoadGLLoader(SDL_GL_GetProcAddress);
    log_info("Vendor:   %s", glGetString(GL_VENDOR));
    log_info("Renderer: %s", glGetString(GL_RENDERER));
    log_info("Version:  %s", glGetString(GL_VERSION));

    // Enable the debug callback
    GLint flags;
    glGetIntegerv(GL_CONTEXT_FLAGS, &flags);
    if (flags & GL_CONTEXT_FLAG_DEBUG_BIT) {
        glEnable(GL_DEBUG_OUTPUT);
        glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
        glDebugMessageCallback(gl_debug_output, nullptr);
        glDebugMessageControl(
                GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, NULL, true
        );
    }

    // Use v-sync
    SDL_GL_SetSwapInterval(1);

    // Set OpenGL viewport
    int w, h;
    SDL_GetWindowSize(window, &w, &h);
    glViewport(0, 0, w, h);

    glEnable(GL_DEPTH_TEST);

    // Setup ImGui binding
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui_ImplSDL2_InitForOpenGL(window, glContext);
    ImGui_ImplOpenGL3_Init("#version 130");

    ImGuiIO &io = ImGui::GetIO();
    ImGui::StyleColorsDark();

    // load systems
    FluidSim2DConfig config = {};

    config.sizeX = 128;
    config.sizeY = 128;
    config.particlesPerCellSqrt = 2;
    config.dt = 0.005;
    config.dx = 0.01;
    config.rho = 997.0;
    config.gravityX = 0.0;
    config.gravityY = -9.81;
    config.mode = FS_PICFLIP;
    config.picFlipAlpha = 1.0;
    config.initialValues = new FluidCellType[config.sizeX*config.sizeY];

    for (size_t j = 0; j < config.sizeY; j++) {
        for (size_t i = 0; i < config.sizeX; i++) {
            if (i == 0 || i == config.sizeX - 1 ||
                j == 0 || j == config.sizeY - 1) {
                config.initialValues[i*config.sizeY + j] = FS_SOLID;
            }
            else if (i + j < config.sizeY * 3 / 4) {
                config.initialValues[i*config.sizeY + j] = FS_FLUID;
            }
            else {
                config.initialValues[i*config.sizeY + j] = FS_EMPTY;
            }
        }
    }

    fluidSim2D = FluidSim2D::create(config);
    camera2d = Camera2D::create(&settings, aml::toFloat(fluidSim2D.getGridCenter()));
    camera2d.zoom = 64.0f * (0.001 / fluidSim2D.dx);
    fluidRenderer2D = FluidRenderer2D::create(&fluidSim2D, &camera2d);
    fluidRenderer2D.setup();

    delete[] config.initialValues;
}

void App::free() {
    fluidSim2D.saveStats();
    fluidSim2D.free();
    fluidRenderer2D.free();
    SDL_DestroyWindow(window);
}

void App::start() {
    // Program loop
    Uint32 frameTime;
    Uint32 lastFrameTime = SDL_GetTicks();

    // constexpr Uint32 msPerFrame = 16;

    while (!quit) {
        frameTime = SDL_GetTicks();

        // Process input
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT) {
                quit = true;
                break;
            }
            else if (event.type == SDL_KEYDOWN) {
                switch (event.key.keysym.sym) {
                    case SDLK_ESCAPE:
                        quit = true;
                        break;
                }
            }
        }

        //
        // Update
        //
        float dt = (float)(frameTime - lastFrameTime) / 1000.f;
        lastFrameTime = frameTime;

        InputManager::get()->update();
        camera2d.update(dt);
        fluidSim2D.update();
        fluidRenderer2D.update();

        //
        // Render
        //
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame(window);
        ImGui::NewFrame();

        // clear screen
        glViewport(0, 0, (int)ImGui::GetIO().DisplaySize.x, (int)ImGui::GetIO().DisplaySize.y);
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // draw
        fluidRenderer2D.draw();

        // draw UI
        fluidRenderer2D.drawUI();
        camera2d.drawUI();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // screenshot
        static int imageNum = 0;
        auto inputMgr = InputManager::get();
        if (inputMgr->isKeyEntered(SDL_SCANCODE_P)) {
            auto name = String::fmt("Screenshot %d.bmp", imageNum).unwrap();
            screenshot(name.data());
            name.free();
            imageNum++;
        }

        SDL_GL_SwapWindow(window);

        /*
        int msPerFrame = 16;
        Uint32 totalFrameTime = SDL_GetTicks() - frameTime;
        if (totalFrameTime < msPerFrame) {
            SDL_Delay(msPerFrame - (SDL_GetTicks() - frameTime));
        }
         */
    }
}

void ByteSwap(uint8_t* a, uint8_t* b)
{
    uint8_t tmp = *b;
    *b = *a;
    *a = tmp;
}

void App::screenshot(const char* filename) {

    // Press P for screenshot

    {
        uint8_t* pixels = new uint8_t[3 * settings.screenSize.x * settings.screenSize.y];
        glReadPixels(0, 0, settings.screenSize.x, settings.screenSize.y, GL_RGB, GL_UNSIGNED_BYTE, pixels);

        // 1. Exchange red / blue pixel info.
        for (int i = 0; i < settings.screenSize.x * settings.screenSize.y; i++) // Exchange red / blue mask.
            ByteSwap(&pixels[3 * i], &pixels[3 * i + 2]);

        // 2. Change top <-> down order.
        for (int row = 0; row < settings.screenSize.y / 2; row++)
        {
            int oppoRow = settings.screenSize.y - 1 - row;

            if (row >= oppoRow) break;

            for (int col = 0; col < settings.screenSize.x; col++)
            {
                for(int i = 0; i < 3; i++)
                    ByteSwap(&pixels[3 * (settings.screenSize.x * row + col) + i], &pixels[3 * (settings.screenSize.x * oppoRow + col) + i]);
            }
        }

        SDL_Surface* surface;
        surface = SDL_CreateRGBSurfaceFrom(pixels, settings.screenSize.x, settings.screenSize.y, 24, 3 * settings.screenSize.x, 0, 0, 0, 0);
        SDL_SaveBMP(surface, filename);
        SDL_FreeSurface(surface);
        delete[] pixels;
    }
}
