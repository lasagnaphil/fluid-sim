//
// Created by lasagnaphil on 7/10/18.
//

#include "InputManager.h"

InputManager InputManager::inst = {};

void InputManager::update() {
    // update keyboard state
    memcpy(prevKeys, currKeys, SDL_NUM_SCANCODES * sizeof(Uint8));
    memcpy(currKeys, SDL_GetKeyboardState(NULL), SDL_NUM_SCANCODES * sizeof(Uint8));

    // update mouse state
    prevMouse = currMouse;
    prevMousePos = currMousePos;
    currMouse = SDL_GetMouseState(&currMousePos.x, &currMousePos.y);
}

bool InputManager::isKeyPressed(SDL_Scancode key) {
    return currKeys[key];
}

bool InputManager::isKeyEntered(SDL_Scancode key) {
    return currKeys[key] && !prevKeys[key];
}

bool InputManager::isKeyExited(SDL_Scancode key) {
    return !currKeys[key] && prevKeys[key];
}

bool InputManager::isMousePressed(Uint8 button) {
    return (currMouse & SDL_BUTTON(button)) != 0;
}

bool InputManager::isMouseEntered(Uint8 button) {
    return (currMouse & SDL_BUTTON(button)) != 0 && (prevMouse & SDL_BUTTON(button)) == 0;
}

bool InputManager::isMouseExited(Uint8 button) {
    return (currMouse & SDL_BUTTON(button)) == 0 && (prevMouse & SDL_BUTTON(button)) != 0;
}

vec2i InputManager::getMousePos() {
    return currMousePos;
}

vec2i InputManager::getRelMousePos() {
    vec2i pos;
    SDL_GetRelativeMouseState(&pos.x, &pos.y);
    return pos;
}

