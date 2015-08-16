#version 330

uniform sampler2D tex;
uniform vec3 color;
uniform int mode;

in vec2 uv;

void main()
{    

    switch(mode)
    {
        case 0:
            gl_FragColor.rgb = color;
            gl_FragColor.a = 1.0f;
            break;
        case 1:
            gl_FragColor.rgb = texture(tex, uv).r*color;
            gl_FragColor.a = texture(tex, uv).r;
            break;
    }
}
