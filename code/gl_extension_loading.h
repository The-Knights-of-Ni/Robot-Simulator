#ifndef GL_EXTENSION_LOADING
#define GL_EXTENSION_LOADING

#include <SDL.h>
#include <SDL_opengl.h>

#define declareGLFunction(rval, ext, args) typedef rval (APIENTRY * ext##_Func)args; ext##_Func ext = 0;

//opengl functions that require an HDC to get a pointer to
declareGLFunction(void, glDebugMessageCallbackARB, (GLDEBUGPROCARB callback, const void* userParam));

declareGLFunction(void, glGenBuffers, (GLsizei n, const GLuint * buffers));
declareGLFunction(void, glBufferData, (GLenum target, GLsizei ptrsize, const GLvoid * data, GLenum usage));
declareGLFunction(void, glBindBuffer, (GLenum target, GLuint buffer));
declareGLFunction(void, glVertexAttribPointer, (GLuint index, GLint size, GLenum type, GLboolean normalized, GLsizei stride, const GLvoid * pointer));
declareGLFunction(void, glEnableVertexAttribArray, (GLuint index));
declareGLFunction(void, glGenVertexArrays, (GLsizei n, GLuint *arrays));
declareGLFunction(void, glBindVertexArray, (GLuint array));

declareGLFunction(GLuint, glCreateShader, (GLenum shaderType));
declareGLFunction(void, glShaderSource, (GLuint shader, GLsizei count, const GLchar * const *string, const GLint *length));
declareGLFunction(void, glCompileShader, (GLuint shader));
declareGLFunction(void, glGetShaderiv, (GLuint shader, GLenum pname, GLint *params));
declareGLFunction(void, glGetShaderInfoLog, (GLuint shader, GLsizei maxLength, GLsizei *length, GLchar *infoLog));
declareGLFunction(GLuint, glCreateProgram, (void));
declareGLFunction(void, glAttachShader, (GLuint program, GLuint shader));
declareGLFunction(void, glLinkProgram, (GLuint program));
declareGLFunction(void, glGetProgramiv, (GLuint program, GLenum pname, GLint *params));
declareGLFunction(void, glGetProgramInfoLog, (GLuint program, GLsizei maxLength, GLsizei *length, GLchar *infoLog));
declareGLFunction(void, glDetachShader, (GLuint program, GLuint shader));
declareGLFunction(void, glDeleteShader, (GLuint shader));
declareGLFunction(void, glUseProgram, (GLuint program));

declareGLFunction(void, glActiveTexture, (GLenum texture));

declareGLFunction(GLuint, glGetUniformLocation, (GLuint program, const GLchar * name));
declareGLFunction(void, glUniform1i, (GLuint location, GLint v0));
declareGLFunction(void, glUniform2f, (GLuint location, GLfloat v0, GLfloat v1));
declareGLFunction(void, glUniformMatrix4fv, (GLint location, GLsizei count, GLboolean transpose, const GLfloat *value));

void loadGLFunctions()
{
    glDebugMessageCallbackARB = (glDebugMessageCallbackARB_Func) SDL_GL_GetProcAddress("glDebugMessageCallbackARB");

    glGenBuffers = (glGenBuffers_Func) SDL_GL_GetProcAddress("glGenBuffers");
    glBufferData = (glBufferData_Func) SDL_GL_GetProcAddress("glBufferData");
    glBindBuffer = (glBindBuffer_Func) SDL_GL_GetProcAddress("glBindBuffer");
    glVertexAttribPointer = (glVertexAttribPointer_Func) SDL_GL_GetProcAddress("glVertexAttribPointer");
    glEnableVertexAttribArray = (glEnableVertexAttribArray_Func) SDL_GL_GetProcAddress("glEnableVertexAttribArray");
    glGenVertexArrays = (glGenVertexArrays_Func) SDL_GL_GetProcAddress("glGenVertexArrays");
    glBindVertexArray = (glBindVertexArray_Func) SDL_GL_GetProcAddress("glBindVertexArray");

    glCreateShader = (glCreateShader_Func) SDL_GL_GetProcAddress("glCreateShader");
    glShaderSource = (glShaderSource_Func) SDL_GL_GetProcAddress("glShaderSource");
    glCompileShader = (glCompileShader_Func) SDL_GL_GetProcAddress("glCompileShader");
    glGetShaderiv = (glGetShaderiv_Func) SDL_GL_GetProcAddress("glGetShaderiv");
    glGetShaderInfoLog = (glGetShaderInfoLog_Func) SDL_GL_GetProcAddress("glGetShaderInfoLog");
    glCreateProgram = (glCreateProgram_Func) SDL_GL_GetProcAddress("glCreateProgram");
    glAttachShader = (glAttachShader_Func) SDL_GL_GetProcAddress("glAttachShader");
    glLinkProgram = (glLinkProgram_Func) SDL_GL_GetProcAddress("glLinkProgram");
    glGetProgramiv = (glGetProgramiv_Func) SDL_GL_GetProcAddress("glGetProgramiv");
    glGetProgramInfoLog = (glGetProgramInfoLog_Func) SDL_GL_GetProcAddress("glGetProgramInfoLog");
    glDetachShader = (glDetachShader_Func) SDL_GL_GetProcAddress("glDetachShader");
    glDeleteShader = (glDeleteShader_Func) SDL_GL_GetProcAddress("glDeleteShader");
    glUseProgram = (glUseProgram_Func) SDL_GL_GetProcAddress("glUseProgram");

    glActiveTexture = (glActiveTexture_Func) SDL_GL_GetProcAddress("glActiveTexture");

    glGetUniformLocation = (glGetUniformLocation_Func) SDL_GL_GetProcAddress("glGetUniformLocation");
    glUniform1i = (glUniform1i_Func) SDL_GL_GetProcAddress("glUniform1i");
    glUniform2f = (glUniform2f_Func) SDL_GL_GetProcAddress("glUniform2f");
    glUniformMatrix4fv = (glUniformMatrix4fv_Func) SDL_GL_GetProcAddress("glUniformMatrix4fv");

    //TODO: I should probably actualy check these and give a message that the function could not be loaded
    assert(glDebugMessageCallbackARB);

    assert(glGenBuffers != 0);
    assert(glBufferData != 0);
    assert(glBindBuffer != 0);
    assert(glVertexAttribPointer);
    assert(glEnableVertexAttribArray);
    assert(glGenVertexArrays);
    assert(glBindVertexArray);

    assert(glCreateShader != 0);
    assert(glShaderSource != 0);
    assert(glCompileShader);
    assert(glAttachShader);
    assert(glLinkProgram);
    assert(glGetProgramiv);
    assert(glGetProgramInfoLog);
    assert(glDetachShader);
    assert(glUseProgram);
    
    assert(glActiveTexture);

    assert(glGetUniformLocation);
    assert(glUniform1i);
    assert(glUniform2f);
    assert(glUniformMatrix4fv);
}
#endif
