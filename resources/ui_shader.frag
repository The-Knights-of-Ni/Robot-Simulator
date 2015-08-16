#version 330

uniform sampler2D tex;

in vec2 uv;

void main()
{    
    gl_FragColor.rgb = texture(tex, uv).r*vec3(0.0, 0.0, 0.0);
    gl_FragColor.a = texture(tex, uv).r;
}
