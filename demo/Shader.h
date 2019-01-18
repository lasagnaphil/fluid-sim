//
// Created by lasagnaphil on 2017-03-27.
//

#pragma once

#include <glad/glad.h>
#include "Storage.h"
#include "StringPool.h"

#include <mat4f.h>
#include <vec2.h>
#include <vec3.h>
#include <vec4f.h>

struct Shader {
    GLuint id;
    Symbol vertexPath;
    Symbol fragmentPath;

    static Shader create(const char* vertexPath, const char* fragmentPath);
    static Shader fromStr(const char* vertexStr, const char* fragmentStr);

    void use();
    void setBool(const char* name, bool value);
    void setInt(const char* name, int value);
    void setFloat(const char* name, float value);
    void setMatrix4(const char* name, const mat4f& value);
    void setVector2(const char* name, const vec2f& value);
    void setVector3(const char* name, const vec3f& value);
    void setVector4(const char* name, const vec4f& value);

    void deserialize(json::Value json);
    json::Value serialize();
};


