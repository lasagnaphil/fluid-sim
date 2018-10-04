#version 330 core

layout (location = 0) in vec3 inPos;
layout (location = 1) in vec3 inOffset;

out vec4 color;

uniform mat4 proj;
uniform mat4 view;

void main() {
    gl_Position = proj * view * vec4(inPos + inOffset, 1.0);
    color = vec4(0.0, 0.0, 1.0, 0.5);
}
