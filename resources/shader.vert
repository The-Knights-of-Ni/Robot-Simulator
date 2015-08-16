#version 330

layout (location = 0) in vec3 p;

uniform mat4x4 t;

void main()
{
    gl_Position.xyz = p;
    gl_Position.w = 1.0;
    gl_Position = gl_Position*t;
}
