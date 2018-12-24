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

#include "InputManager.h"

#include "WaterSim3D.h"
#include "WaterSim2D.h"
#include "WaterRenderer2D.h"
#include "WaterRenderer3D.h"

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

void App::init(mathfu::vec2i screenSize, Mode mode) {
    settings.screenSize = screenSize;
    this->mode = mode;

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
    if (mode == Mode::Dim2) {
        waterSim2D = new WaterSim2D();
        waterSim2D->setup(WaterSimSettings::Dim2D::DT, WaterSimSettings::Dim2D::DX, 997.0, -9.81);
        camera2d = Camera2D::create(&settings, vec2f(waterSim2D->getGridCenter()));
        camera2d.zoom = 64.0f * (0.001 / waterSim2D->dx);
        waterRenderer2D = new WaterRenderer2D();
        waterRenderer2D->setup(waterSim2D, &camera2d);
    }
    else if (mode == Mode::Dim3) {
        waterSim3D = new WaterSim3D();
        waterSim3D->setup();
        fpsCamera = FirstPersonCamera::create(&settings);
        fpsCamera.transform.pos = vec3f(0.5f, 0.5f, 3.0f);
        waterRenderer3D = new WaterRenderer3D();
        waterRenderer3D->setup(waterSim3D, &fpsCamera);
    }
}

void App::free() {
    if (mode == Mode::Dim2) {
        waterSim2D->free();
        delete waterSim2D;
        delete waterRenderer2D;
    }
    else if (mode == Mode::Dim3) {
        delete waterSim3D;
        delete waterRenderer3D;
    }
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
        if (mode == Mode::Dim2) {
            camera2d.update(dt);
            waterSim2D->update();
            waterRenderer2D->update();
        }
        else if (mode == Mode::Dim3) {
            fpsCamera.update(dt);
            waterSim3D->update();
            waterRenderer3D->update();
        }

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
        if (mode == Mode::Dim2) {
            waterRenderer2D->draw();
        }
        else if (mode == Mode::Dim3) {
            waterRenderer3D->draw();
        }

        // draw UI
        if (mode == Mode::Dim2) {
            waterRenderer2D->drawUI();
            camera2d.drawUI();
        }
        else if (mode == Mode::Dim3) {
            waterRenderer3D->drawUI();
            fpsCamera.drawUI();
        }

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        SDL_GL_SwapWindow(window);

        /*
        Uint32 totalFrameTime = SDL_GetTicks() - frameTime;
        if (totalFrameTime < msPerFrame) {
            SDL_Delay(msPerFrame - (SDL_GetTicks() - frameTime));
        }
         */
    }
}