#version 330

void main()
{
    gl_FragColor.rgb = pow(gl_FragCoord.z, 1.0/2.2)*vec3(1.0, 1.0, 1.0);
    gl_FragColor.a = 1.0;
}
