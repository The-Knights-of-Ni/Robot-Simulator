#version 330

uniform sampler2D tex;
uniform vec3 color;
uniform int mode;

in vec2 uv;

out vec4 frag_color;

void main()
{    

    switch(mode)
    {
        case 0:
            frag_color.rgb = color;
            frag_color.a = 1.0f;
            break;
        case 1:
            frag_color.rgb = texture(tex, uv).r*color;
            frag_color.a = texture(tex, uv).r;
            break;
    }
}
