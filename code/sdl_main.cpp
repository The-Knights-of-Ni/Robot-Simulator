#include "misc.h"

#include <stdio.h>
#include <SDL.h>

int main(int n_arg, char * args[])
{
    SDL_Init(SDL_INIT_VIDEO);
    
    SDL_Window * window = SDL_CreateWindow("Simulator",
                                           SDL_WINDOWPOS_UNDEFINED,
                                           SDL_WINDOWPOS_UNDEFINED,
                                           640,
                                           480,
                                           SDL_WINDOW_OPENGL|SDL_WINDOW_RESIZABLE);
    
    if (window == NULL)
    {
        printf("Could not create window: %s\n", SDL_GetError());
        return 1;
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
    }

    
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
