#version 330 core

layout (location = 0) in vec3 inPos;
layout (location = 1) in vec3 inOffset;

out vec4 color;

uniform mat4 proj;
uniform mat4 view;
/*
uniform int sizeX;
uniform int sizeY;
uniform float cellSize;
*/

void main() {
/*
    int z = gl_instanceID / sizeX*sizeY;
    int y = gl_instanceID / sizeX - sizeY * z;
    int x = gl_instanceID - sizeX*sizeY*z - sizeX*y;

    gl_Position = projection * view * vec4(inPos + cellSize * vec3(x, y, z), 1.0);
*/
    gl_Position = proj * view * vec4(inPos + inOffset, 1.0);
    color = vec4(0.0, 0.0, 1.0, 0.5);
}
