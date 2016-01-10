#ifndef UI
#define UI

//#include "meth,h"

//default window size
static int window_width = 640;
static int window_height = 480;
static float wx_scale = 2.0/window_width;
static float wy_scale = 2.0/window_height;

// void * STBTT_malloc_func(size_t size, void ** free_memory)
// {
//     void * temp =(*((void **) (free_memory)));
//     (*(free_memory)) = (void *) ((char *) (*free_memory) + size);
//     return temp;
// }
//#define STBTT_malloc(size, free_memory) STBTT_malloc_func(size, (void **) free_memory)
//#define STBTT_free(x, u) ((void) x, (void) u)
#define STBTT_assert(x) assert(x)
#define STB_RECT_PACK_IMPLEMENTATION
#include <stb_rect_pack.h>
#define STB_TRUETYPE_IMPLEMENTATION
#include <stb_truetype.h>
#include <stdio.h>
#include <SDL.h>

bool left_click = 0;
bool prev_left_click = 0;

#define font_bitmap_width 128
#define font_bitmap_height 256

static stbtt_packedchar * font_pack_data;
static stbtt_fontinfo font_info;

int font_ascent;
int font_descent;
int font_line_gap;

static GLuint ui_uv0;
static GLuint ui_uv1;
static GLuint ui_c0;
static GLuint ui_c1;
static GLuint ui_color;
static GLuint ui_mode;

float getTextWidthInWindowPixles(char * s)
{
    float width = 0.0;
    for(;; s++)
    {
        stbtt_packedchar & c_data = font_pack_data[*s-32];
        
        width += c_data.xadvance;
        if(*(s+1))
        {
            float kern_advance = stbtt_ScaleForPixelHeight(&font_info, 16)*stbtt_GetCodepointKernAdvance(&font_info, *s, *(s+1));
            width += kern_advance;
        }
        else break;
    }
    return width;
}

float getTextWidthInWindowUnits(char * s)
{
    return getTextWidthInWindowPixles(s)*wx_scale;
}

void drawText(float px0, float py0, char * s)
{
    //glBegin(GL_QUADS);
    for(;; s++)
    {
        stbtt_packedchar & c_data = font_pack_data[*s-32];
                
        float ibw = 1.0/font_bitmap_width;
        float ibh = 1.0/font_bitmap_height;
        
        float x0 = (c_data.x0-0.5)*ibw;
        float y0 = (c_data.y0-0.5)*ibh;
        float x1 = c_data.x1*ibw;
        float y1 = c_data.y1*ibh;
        //printf("%c, %f, %f, %f, %f\n", *s, x0, y0, x1, y1);
                
        float wx0 = px0 + (c_data.xoff-0.5)*wx_scale;
        float wy0 = py0 - (c_data.yoff-0.5)*wy_scale;
        float wx1 = px0 + c_data.xoff2*wx_scale;
        float wy1 = py0 - c_data.yoff2*wy_scale;
                
        // glTexCoord2f(x1, y1);
        // glVertex2f(wx1, wy1);
        // glTexCoord2f(x0, y1);
        // glVertex2f(wx0, wy1);
        // glTexCoord2f(x0, y0);
        // glVertex2f(wx0, wy0);
        // glTexCoord2f(x1, y0);
        // glVertex2f(wx1, wy0);

        glUniform2f(ui_uv0, x0, y0);
        glUniform2f(ui_uv1, x1, y1);
        glUniform2f(ui_c0, wx0, wy0);
        glUniform2f(ui_c1, wx1, wy1);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
                
        if(*(s+1))
        {
            float kern_advance = stbtt_ScaleForPixelHeight(&font_info, 16)*stbtt_GetCodepointKernAdvance(&font_info, *s, *(s+1));
            px0 += (c_data.xadvance + kern_advance)*wx_scale;
        }
        else break;
    }
    //glEnd();
}

float getButtonHeight(float y_padding)
{
    return (stbtt_ScaleForPixelHeight(&font_info, 16)*wy_scale*(
                          font_ascent -
                          font_descent)+y_padding*2);
}

bool doButtonNW(char * string, float x0, float y1, float x_padding, float y_padding)
{    
    float width = getTextWidthInWindowUnits(string);            
    
    x_padding *= wx_scale;
    y_padding *= wy_scale;
    
    int x;
    int y;
    SDL_GetMouseState(&x, &y);
    float mx = x*wx_scale-1.0;
    float my = -(y*wy_scale-1.0);
    
    float y0 = y1 - (stbtt_ScaleForPixelHeight(&font_info, 16)*wy_scale*(
                          font_ascent -
                          font_descent)+y_padding*2);
    float x1 = x0+width+x_padding*2;
    
    glUniform1i(ui_mode, 0);
    bool over = mx > x0 && mx < x1 && my > y0 && my < y1;
    if(over) glUniform3f(ui_color, 1.0, 1.0, 1.0);
    else glUniform3f(ui_color, 0.5, 0.5, 0.5);
    glUniform2f(ui_uv0, 0, 0);
    glUniform2f(ui_uv1, 0, 0);
    glUniform2f(ui_c0, x0, y0);
    glUniform2f(ui_c1, x1, y1);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    
    glUniform1i(ui_mode, 1);
    glUniform3f(ui_color, 0.0, 0.0, 0.0);
    drawText(x0+x_padding, y1-(stbtt_ScaleForPixelHeight(&font_info, 16)*wy_scale*font_ascent+y_padding), string);
    
    return over && !left_click && prev_left_click;
}

struct virtual_joystick
{
    v2f joystick;
    bool held;
};
virtual_joystick doVirtualJoystickNW(bool held, float x0, float y1, float width, float height)
{    
    float y0 = y1 - height*wy_scale;
    float x1 = x0 + width *wx_scale;
    
    int x;
    int y;
    SDL_GetMouseState(&x, &y);
    float mx = x*wx_scale-1.0;
    float my = -(y*wy_scale-1.0);
    
    glUniform1i(ui_mode, 0);
    bool over = mx > x0 && mx < x1 && my > y0 && my < y1;
    
    if(left_click)
    {
        if(over) held = true;
    }
    else
    {
        held = false;
    }
    
    v2f joystick = {};
    if(held)
    {
        joystick.x = clamp(2*((mx-x0)/(x1-x0)-0.5), -1.0, 1.0);
        joystick.y = clamp(2*((my-y0)/(y1-y0)-0.5), -1.0, 1.0);
    }
    
    if(over) glUniform3f(ui_color, 1.0, 1.0, 1.0);
    else glUniform3f(ui_color, 0.5, 0.5, 0.5);
    glUniform2f(ui_uv0, 0, 0);
    glUniform2f(ui_uv1, 0, 0);
    glUniform2f(ui_c0, x0, y0);
    glUniform2f(ui_c1, x1, y1);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    
    //draw joystick dot
    float dot_size = 8;
    glUniform3f(ui_color, 0.0, 0.0, 0.0);
    glUniform2f(ui_uv0, 0, 0);
    glUniform2f(ui_uv1, 0, 0);
    glUniform2f(ui_c0, (x1-x0)/2*joystick.x+(x0+x1)/2-dot_size*wx_scale, (y1-y0)/2*joystick.y+(y0+y1)/2-dot_size*wy_scale);
    glUniform2f(ui_c1, (x1-x0)/2*joystick.x+(x0+x1)/2+dot_size*wx_scale, (y1-y0)/2*joystick.y+(y0+y1)/2+dot_size*wy_scale);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    
    return {joystick, held};
}
#endif
