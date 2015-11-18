#version 330

uniform int mode;

void main()
{
    if(mode == 0)
    {
        gl_FragColor.rgb = pow(gl_FragCoord.z, 1.0/2.2)*vec3(1.0, 1.0, 1.0);
        gl_FragColor.a = 1.0;
    }
    else
    {
        gl_FragColor.rgb = pow(gl_FragCoord.z, 1.0/2.2)*vec3(0.0, 1.0, 0.0);
        gl_FragColor.a = 1.0;
    }
}
