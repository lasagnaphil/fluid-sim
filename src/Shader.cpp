//
// Created by lasagnaphil on 2017-03-27.
//

#include <StringPool.h>
#include "Shader.h"
#include "log.h"

using namespace mathfu;

GLuint compile_shader(GLenum type, const GLchar *source) {
    GLuint shader = glCreateShader(type);

    glShaderSource(shader, 1, &source, NULL);
    glCompileShader(shader);
    GLint status;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &status);

    if (status == GL_TRUE) {
        log_info("Shader (type %d) is compiled successfully!", type);
    } else {
        log_info("Shader (type %d) compile failed!", type);
    }

    char compileInfo[512];
    glGetShaderInfoLog(shader, 512, NULL, compileInfo);
    log_info("Compile Log: \n%s\n", compileInfo);

    return shader;
}

Shader Shader::create(const char* vertexPath, const char* fragmentPath) {
    File vertexFile = File::open(vertexPath, "rb+").unwrap();
    String vertexCode = vertexFile.readAll().unwrap();
    defer {vertexCode.free();};
    vertexFile.close();
    File fragmentFile = File::open(fragmentPath, "rb+").unwrap();
    String fragmentCode = fragmentFile.readAll().unwrap();
    defer {fragmentCode.free();};
    fragmentFile.close();
    return Shader::fromStr(vertexCode.data(), fragmentCode.data());
}

Shader Shader::fromStr(const char* vertexStr, const char* fragmentStr) {
    Shader shader;
    shader.id = glCreateProgram();
    shader.vertexPath = SYM("<memory>");
    shader.fragmentPath = SYM("<memory>");

    GLuint vertexShaderPtr = compile_shader(GL_VERTEX_SHADER, vertexStr);
    GLuint fragmentShaderPtr = compile_shader(GL_FRAGMENT_SHADER, fragmentStr);
    glAttachShader(shader.id, vertexShaderPtr);
    glAttachShader(shader.id, fragmentShaderPtr);
    glLinkProgram(shader.id);
    GLint success;
    GLchar infoLog[512];
    glGetProgramiv(shader.id, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(shader.id, 512, NULL, infoLog);
        log_error("Error: shader program linking failed");
    }

    glDeleteShader(vertexShaderPtr);
    glDeleteShader(fragmentShaderPtr);

    return shader;
}

void Shader::use() {
    glUseProgram(id);
}

void Shader::setBool(const char* name, bool value) {
    glUniform1i(glGetUniformLocation(id, name), (int)value);
}

void Shader::setInt(const char* name, int value) {
    glUniform1i(glGetUniformLocation(id, name), value);
}

void Shader::setFloat(const char* name, float value) {
    glUniform1f(glGetUniformLocation(id, name), value);
}

void Shader::setMatrix4(const char *name, const mat4f& value) {
    glUniformMatrix4fv(glGetUniformLocation(id, name), 1, GL_FALSE, (const GLfloat*) &value);
}

void Shader::setVector2(const char *name, const vec2f& value) {
    glUniform2fv(glGetUniformLocation(id, name), 1, (const GLfloat*) &value.data_);
}

void Shader::setVector3(const char* name, const vec3f& value) {
    glUniform3fv(glGetUniformLocation(id, name), 1, (const GLfloat*) &value.data_);
}

void Shader::setVector4(const char *name, const vec4f& value) {
    glUniform4fv(glGetUniformLocation(id, name), 1, (const GLfloat*) &value.data_);
}

void Shader::deserialize(json::Value json) {
    id = json.get("id").asInt().unwrap();
    vertexPath = SYM(json.get("vertexPath").asStr().unwrap());
    fragmentPath = SYM(json.get("fragmentPath").asStr().unwrap());
}

json::Value Shader::serialize() {
    json::Value json = json::Value::createObject();
    json.addField("id", (int)id);
    json.addField("vertexPath", StringPool::inst.getStr(vertexPath).data);
    json.addField("fragmentPath", StringPool::inst.getStr(fragmentPath).data);
    return json;
}

