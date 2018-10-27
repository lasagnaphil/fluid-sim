//
// Created by lasagnaphil on 2017-03-27.
//

#pragma once

#include <glad/glad.h>
#include <mathfu/glsl_mappings.h>
#include "Storage.h"
#include "StringPool.h"

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
    void setMatrix4(const char* name, const mathfu::mat4f& value);
    void setVector2(const char* name, const mathfu::vec2f& value);
    void setVector3(const char* name, const mathfu::vec3f& value);
    void setVector4(const char* name, const mathfu::vec4f& value);

    void deserialize(json::Value json);
    json::Value serialize();
};


