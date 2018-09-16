//
// Created by lasagnaphil on 2/6/18.
//

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
#include <StringPool.h>

#include "Image.h"

Image Image::create(const char *filename, int desiredChannels) {
    Image image;
    image.path = SYM(filename);
    image.desiredChannels = desiredChannels;
    image.load();
    return image;
}

void Image::free() {
    if (data) {
        stbi_image_free(data);
    }
}

void Image::load() {
    auto filename = STR(path);
    data = stbi_load(filename.data, &width, &height, &nrChannels, desiredChannels);
    if (!data) {
        log_error("Failed to load image %s!\n", filename.data);
    }
}

void Image::deserialize(json::Value json) {
    const char* pathStr = json.get("path").asStr().unwrap();
    path = SYM(pathStr);
}

json::Value Image::serialize() {
    json::Value json = json::Value::createObject();
    json.addField("path", STR(path).data);
    return json;
}

