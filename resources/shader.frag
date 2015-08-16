#version 330

out vec4 frag_color;

void main()
{
    frag_color.rgb = pow(gl_FragCoord.z, 1.0/2.2)*vec3(1.0, 1.0, 1.0);
    frag_color.a = 1.0;
}
