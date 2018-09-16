//
// Created by lasagnaphil on 7/10/18.
//

#ifndef THESYSTEM_INPUTMANAGER_H
#define THESYSTEM_INPUTMANAGER_H

#include <SDL_events.h>
#include <HandmadeMath.h>
#include "Vec.h"
#include "Map.h"
#include "math/Vector2.h"

struct InputManager {
    Uint8 currKeys[SDL_NUM_SCANCODES] = {};
    Uint8 prevKeys[SDL_NUM_SCANCODES] = {};

    Uint32 currMouse = {};
    Uint32 prevMouse = {};
    Vector2i currMousePos = {};
    Vector2i prevMousePos = {};

    static InputManager inst;
    static InputManager* get() {
        return &InputManager::inst;
    }

    void update();

    bool isKeyPressed(SDL_Scancode key);
    bool isKeyEntered(SDL_Scancode key);
    bool isKeyExited(SDL_Scancode key);

    bool isMousePressed(Uint8 button);
    bool isMouseEntered(Uint8 button);
    bool isMouseExited(Uint8 button);

    Vector2i getMousePos();
    Vector2i getRelMousePos();
};

#endif //THESYSTEM_INPUTMANAGER_H
