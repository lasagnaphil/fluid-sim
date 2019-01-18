//
// Created by lasagnaphil on 2/6/18.
//

#ifndef GENGINE_IMAGE_H
#define GENGINE_IMAGE_H

#include <glad/glad.h>
#include "Storage.h"
#include "StringPool.h"

struct Image {
    unsigned char* data;
    int width, height, nrChannels, desiredChannels;

    Symbol path;

    static Image create(const char* filename, int desiredChannels = 0);
    void free();

    void load();

    void deserialize(json::Value json);
    json::Value serialize();
};

#endif //GENGINE_IMAGE_H
