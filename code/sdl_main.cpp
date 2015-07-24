#include "simulation.h"
#include "misc.h"
#include "gl_extension_loading.h"

#include <stdio.h>
#include <SDL.h>

int readTextFile(char * buffer, const char * filename)
{
    SDL_RWops * file = SDL_RWFromFile(filename,"r");
    if (file)
    {
        int size = SDL_RWsize(file);
        SDL_RWread(file, buffer, size, 1);
        SDL_RWclose(file);
        buffer[size] = 0;
        return size+1;
    }

    printf("error reading file: %s\n", SDL_GetError());
    return 0;
}

GLuint initShader(const char * shader_source, GLenum shader_type)
{
    GLuint shader = glCreateShader(shader_type);
    glShaderSource(shader, 1, &shader_source, 0);
    glCompileShader(shader);

    int error;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &error);
    if(error == 0)
    { //TODO: real error logging
        char info_log[128];
        int info_log_size;
        glGetShaderiv(shader, 0, &info_log_size);
        glGetShaderInfoLog(shader, info_log_size, 0, info_log);
        printf("%s\n", info_log);
        exit(EXIT_FAILURE);
    }

    return shader;
}

int main(int n_arg, char * args[])
{
    SDL_Init(SDL_INIT_VIDEO);

    SDL_Window * window = SDL_CreateWindow("Simulator",
                                           SDL_WINDOWPOS_UNDEFINED,
                                           SDL_WINDOWPOS_UNDEFINED,
                                           640,
                                           480,
                                           SDL_WINDOW_OPENGL|SDL_WINDOW_RESIZABLE);

    if (!window)
    {
        printf("Could not create window: %s\n", SDL_GetError());
        return 1;
    }

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
    SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 32);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    //SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_COMPATIBILITY);

    SDL_GLContext context = SDL_GL_CreateContext(window);
    if(!context)
    {
        printf("Could not create context: %s\n", SDL_GetError());
        return 1;
    }

    loadGLFunctions();

    byte * memory = (byte *) malloc(1*gigabyte);
    byte * free_memory = memory;

    GLuint program;
    GLuint transform_uniform;
    {
        byte * temp_memory = free_memory;
        //TODO: load shaders off disc
        char * vertex_shader_source = (char *) temp_memory;
        readTextFile(vertex_shader_source, "shader.vert");
        GLuint vertex_shader = initShader(vertex_shader_source, GL_VERTEX_SHADER);

        char * fragment_shader_source = (char *) temp_memory;
        int fragment_shader_source_size = readTextFile(vertex_shader_source, "shader.frag");
        temp_memory += fragment_shader_source_size;

        GLuint fragment_shader = initShader(fragment_shader_source, GL_FRAGMENT_SHADER);

        program = glCreateProgram();
        glAttachShader(program, vertex_shader);
        glAttachShader(program, fragment_shader);

        glLinkProgram(program);

        int error;
        glGetProgramiv(program, GL_LINK_STATUS, &error);
        if(error == 0)
        {
            char * info_log = (char *) temp_memory;
            int info_log_size;
            glGetProgramiv(program, 0, &info_log_size);
            glGetProgramInfoLog(program, info_log_size, 0, info_log);
            printf("%s\n", info_log);
            exit(EXIT_FAILURE);
        }

        glDetachShader(program, vertex_shader);
        glDetachShader(program, fragment_shader);
        glDeleteShader(vertex_shader);
        glDeleteShader(fragment_shader);

        transform_uniform = glGetUniformLocation(program, "t");
    }
    {
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        glDepthFunc(GL_LEQUAL);
        glDepthRange(0.0f, 1.0f);
        glEnable(GL_DEPTH_CLAMP);

        glFrontFace(GL_CCW);
        glCullFace(GL_BACK);
        //glEnable(GL_CULL_FACE);

        glLineWidth(1.0);

        glClearColor(0.1, 0.0, 0.2, 1.0);
        glClearDepth(1.0);
    }

    SDL_Event event;

    for ever
    {
        while (SDL_PollEvent(&event)){
            switch (event.type)
            {
                case SDL_QUIT:
                    return 0;
            }
        }

        glClearColor(0.55, 0.082, 0.082, 1.0);
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

        SDL_GL_SwapWindow(window);
    }

    SDL_GL_DeleteContext(context);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
