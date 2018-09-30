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

#define HANDMADE_MATH_IMPLEMENTATION
#include "HandmadeMath.h"

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
        case GL_DEBUG_SOURCE_API:             log_debug("Source: API"); break;
        case GL_DEBUG_SOURCE_WINDOW_SYSTEM:   log_debug("Source: Window System"); break;
        case GL_DEBUG_SOURCE_SHADER_COMPILER: log_debug("Source: Shader Compiler"); break;
        case GL_DEBUG_SOURCE_THIRD_PARTY:     log_debug("Source: Third Party"); break;
        case GL_DEBUG_SOURCE_APPLICATION:     log_debug("Source: Application"); break;
        case GL_DEBUG_SOURCE_OTHER:           log_debug("Source: Other"); break;
    }

    switch (type)
    {
        case GL_DEBUG_TYPE_ERROR:               log_debug("Type: Error"); break;
        case GL_DEBUG_TYPE_DEPRECATED_BEHAVIOR: log_debug("Type: Deprecated Behaviour"); break;
        case GL_DEBUG_TYPE_UNDEFINED_BEHAVIOR:  log_debug("Type: Undefined Behaviour"); break;
        case GL_DEBUG_TYPE_PORTABILITY:         log_debug("Type: Portability"); break;
        case GL_DEBUG_TYPE_PERFORMANCE:         log_debug("Type: Performance"); break;
        case GL_DEBUG_TYPE_MARKER:              log_debug("Type: Marker"); break;
        case GL_DEBUG_TYPE_PUSH_GROUP:          log_debug("Type: Push Group"); break;
        case GL_DEBUG_TYPE_POP_GROUP:           log_debug("Type: Pop Group"); break;
        case GL_DEBUG_TYPE_OTHER:               log_debug("Type: Other"); break;
    }

    switch (severity)
    {
        case GL_DEBUG_SEVERITY_HIGH:         log_debug("Severity: high"); break;
        case GL_DEBUG_SEVERITY_MEDIUM:       log_debug("Severity: medium"); break;
        case GL_DEBUG_SEVERITY_LOW:          log_debug("Severity: low"); break;
        case GL_DEBUG_SEVERITY_NOTIFICATION: log_debug("Severity: notification"); break;
    }
}

void App::init(Vector2i screenSize) {
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
            "Dynamic Desert",
            SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
            screenSize.x, screenSize.y, SDL_WINDOW_OPENGL
    );

    SDL_SetRelativeMouseMode(SDL_TRUE);

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
    waterSim = new WaterSim3D();
    waterRenderer = new WaterRenderer3D();

    camera = FirstPersonCamera::create(&settings);
    camera.transform.pos = HMM_Vec3(0.3f, 0.3f, 1.8f);

    waterSim->setup();
    waterRenderer->setup(waterSim, &camera);
}

void App::free() {
    delete waterSim;
    delete waterRenderer;
    SDL_DestroyWindow(window);
}

void App::start() {
    // Program loop
    Uint32 frameTime;
    Uint32 lastFrameTime = SDL_GetTicks();

    constexpr Uint32 msPerFrame = 16;

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
                    case SDLK_m: {
                        settings.isMouseRelative = !settings.isMouseRelative;
                        SDL_SetRelativeMouseMode(settings.isMouseRelative? SDL_TRUE : SDL_FALSE);
                        camera.mouseMovementEnabled = settings.isMouseRelative;
                        break;
                    }
                }
            }
        }

        //
        // Update
        //
        float dt = (float)(frameTime - lastFrameTime) / 1000.f;
        lastFrameTime = frameTime;

        InputManager::get()->update();
        camera.update(dt);
        waterSim->update();
        waterRenderer->update();

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

        waterRenderer->draw();

        waterRenderer->drawUI();

        ImGui::Begin("Camera Info");
        ImGui::Text("pos: %f %f %f", camera.transform.pos.X, camera.transform.pos.Y, camera.transform.pos.Z);
        ImGui::Text("rot: %f %f %f %f", camera.transform.rot.X, camera.transform.rot.Y, camera.transform.rot.Z, camera.transform.rot.W);
        ImGui::Text("scale: %f %f %f", camera.transform.scale.X, camera.transform.scale.Y, camera.transform.scale.Z);
        ImGui::Text("pitch: %f, yaw: %f", camera.pitch, camera.yaw);
        ImGui::End();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        SDL_GL_SwapWindow(window);

        Uint32 totalFrameTime = SDL_GetTicks() - frameTime;
        if (totalFrameTime < msPerFrame) {
            SDL_Delay(msPerFrame - (SDL_GetTicks() - frameTime));
        }
    }
}