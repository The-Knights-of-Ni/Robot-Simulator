/*
Renderer.h - Handles interfacing with gl fragments and vertices
*/
#ifndef RENDERER
#define RENDERER

struct render_command
{
    uint model;
    v3f position;
    v4f orientation;
};

struct id_to_index
{
    struct
    {
        union
        {
            char name[8];
            uint64 id;
        };
    } id;
    uint index;
    uint next; //collision handling
};

render_command * render_list;
uint n_to_render = 0;

#endif
