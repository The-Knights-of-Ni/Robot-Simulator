#version 330

layout (location = 0) in vec2 p;

uniform vec2 uv0;
uniform vec2 uv1;

uniform vec2 c0;
uniform vec2 c1;

smooth out vec2 uv;

void main()
{
    gl_Position.xy = c1*p+c0*(vec2(1.0)-p);
    gl_Position.z = 1.0;
    gl_Position.w = 1.0;
    uv = uv1*p+uv0*(vec2(1.0)-p);
}
