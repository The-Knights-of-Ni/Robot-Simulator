#define USING_SIMULATOR
#include "native_path.h"
#include "simulation.h"
#include "gl_extension_loading.h"
#include "renderer.h"
#include "ui.h"

#include "gl_renderer.h"

#include "Mk3Teleop_robot_state_elements.h"
#include "drive_simulate.h" //TODO: this should be loaded from a dll


void APIENTRY glErrorCallback(GLenum source, GLenum type, uint id, GLenum severity, GLsizei length, const char * message, void * userParam)
{
    printf("message: %s\n", message);
    assert(0);
}

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

int readFile(byte * buffer, const char * filename)
{
    SDL_RWops * file = SDL_RWFromFile(filename,"rb");
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
        printf("error in %s shader\n%s\n", (shader_type == GL_VERTEX_SHADER) ? "vertex" : "fragment", info_log);
        exit(EXIT_FAILURE);
    }

    return shader;
}

struct vi_buffer
{
    uint vb;
    uint ib;
    uint n;
};

inline vi_buffer createVertexAndIndexBuffer(uint vb_size, float * vb_data, uint ib_size, uint16 * ib_data)
{
    vi_buffer out = {};
    glGenBuffers(1, &out.vb);
    glBindBuffer(GL_ARRAY_BUFFER, out.vb);
    glBufferData(GL_ARRAY_BUFFER, vb_size, vb_data, GL_STATIC_DRAW);

    glGenBuffers(1, &out.ib);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, out.ib);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, ib_size, ib_data, GL_STATIC_DRAW);

    out.n = ib_size/sizeof(ib_data[0]);

    return out;
}

enum attrib
{
    attrib_pos,
    attrib_count,
};

inline void bindVertexAndIndexBuffers(uint vb, uint ib)
{
    glBindBuffer(GL_ARRAY_BUFFER, vb);
    glEnableVertexAttribArray(attrib_pos);
    glVertexAttribPointer(attrib_pos, 3, GL_FLOAT, GL_FALSE, 12, 0);
    //glEnableVertexAttribArray(attrib_norm);
    //glVertexAttribPointer(attrib_norm, 3, GL_FLOAT, GL_FALSE, 24, (void *) 12);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ib);
}

int main(int n_arg, char * args[])
{
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK);

    SDL_Window * window = SDL_CreateWindow("Drive Simulator",
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

    printf("%d joysticks found\n", SDL_NumJoysticks());
    if(SDL_NumJoysticks() > 0)
    {
        joystick = SDL_JoystickOpen(0);

        if(joystick)
        {
            printf("Opened Joystick 0\n");
            printf("Name: %s\n", SDL_JoystickNameForIndex(0));
            printf("Number of Axes: %d\n", SDL_JoystickNumAxes(joystick));
            printf("Number of Buttons: %d\n", SDL_JoystickNumButtons(joystick));
            printf("Number of Balls: %d\n", SDL_JoystickNumBalls(joystick));
        }
        else
        {
            printf("could not open joystick");
        }
    }
	/*START FILE IO INFRASTRUCTURE*/
	MotionProfileIO = fopen("MotionProfile.txt", "w");
    Config config;
    const float kWheelbaseWidth = 14.325;

      config.dt = 0.01;
      config.max_acc = 100;
      config.max_jerk = 110;
      config.max_vel = 32.34*0.7;
      // Path name must be a valid Java class name.
      // Description of this auto mode path.
      // Remember that this is for the GO LEFT CASE!
      waypointSequence p(10);
      p.addWaypoint(waypoint(0, 0, 0));
      p.addWaypoint(waypoint(80, 0, 0));
      p.addWaypoint(waypoint(130, -15, pi/4));
      //p.addWaypoint(waypoint(13.75, 9.5, 0.0/* * Math.PI/18.0*/));

      path drivePath = makePath(p, config, kWheelbaseWidth);

      TrajectoryFollower leftTraj;
      TrajectoryFollower rightTraj;

      const float robot_max_velocity = 21;
      leftTraj.setTrajectory(drivePath.go_left_pair.left);
      rightTraj.setTrajectory(drivePath.go_left_pair.right);

      leftTraj.configure(0, 0, 0, 1/robot_max_velocity, 1/100.0, 0);
      rightTraj.configure(0, 0, 0, 1/robot_max_velocity, 1/100.0, 0);
      v4f leftTrajStatus = (v4f) {0,0,0,0};
      v4f rightTrajStatus = (v4f) {0,0,0,0};
      float motionProfileIOTime = 0.0;
      float motionProfileIOdt = 0.01;
      fprintf(MotionProfileIO,"Start of Path: \n");
    /*END FILE IO INFRASTRUCTURE*/
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 32);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_DEBUG_FLAG);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    //SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_COMPATIBILITY);

    SDL_GLContext context = SDL_GL_CreateContext(window);
    if(!context)
    {
        printf("Could not create context: %s\n", SDL_GetError());
        return 1;
    }

    loadGLFunctions();

    #if defined(DEBUG) && defined(_WIN32)
    glDebugMessageCallbackARB(glErrorCallback, 0);
    //glEnable(GL_DEBUG_OUTPUT);
    #endif

    void * memory = (byte *) malloc(0.5*gigabyte);
    void * free_memory = memory;

    //font stuff
    font_pack_data = (stbtt_packedchar *) free_memory;
    free_memory = (void *) ((stbtt_packedchar *) free_memory + 256);

    GLuint ftex;
    {
        stbtt_pack_context pc;

        void * temp_memory = free_memory;

        uint8 * font_bitmap = (uint8 *) temp_memory;
        temp_memory = (void *) ((uint8 *) temp_memory + font_bitmap_width*font_bitmap_height);

        unsigned char * font_data = (unsigned char *) temp_memory;
        uint font_data_size = readFile(font_data, "DroidSans-Bold.ttf");
        temp_memory = (void *) ((unsigned char *) temp_memory + font_data_size);

        //STBTT_DEF int stbtt_InitFont(&font_info, font_data, 0);

        stbtt_PackBegin(&pc, font_bitmap, font_bitmap_width, font_bitmap_height, 0, 1, 0);

        stbtt_pack_range ranges[1];
        ranges[0].font_size = 16.0f;
        ranges[0].first_unicode_codepoint_in_range = 32;
        ranges[0].array_of_unicode_codepoints = 0;
        ranges[0].num_chars = 126-32;
        ranges[0].chardata_for_range = font_pack_data;

        stbtt_PackSetOversampling(&pc, 1, 1);
        //int error = stbtt_PackFontRanges(&pc, font_data, 0, ranges, 1);

        int error;
        int font_index = 0;
        { //modified from stbtt_PackFontRanges in stb_truetype.h so I can get font_info
            int i,j,n;

            stbrp_rect    *rects;

            // flag all characters as NOT packed
            for (i=0; i < len(ranges); ++i)
            {
                for (j=0; j < ranges[i].num_chars; ++j)
                {
                    ranges[i].chardata_for_range[j].x0 = 0;
                    ranges[i].chardata_for_range[j].y0 = 0;
                    ranges[i].chardata_for_range[j].x1 = 0;
                    ranges[i].chardata_for_range[j].y1 = 0;
                }
            }

            n = 0;
            for (i=0; i < len(ranges); ++i)
                n += ranges[i].num_chars;

            rects = (stbrp_rect *) temp_memory; //STBTT_malloc(sizeof(*rects) * n, pc.user_allocator_context);
            if (rects == NULL)
                return 0;

            stbtt_InitFont(&font_info, font_data, stbtt_GetFontOffsetForIndex(font_data, font_index));

            n = stbtt_PackFontRangesGatherRects(&pc, &font_info, ranges, len(ranges), rects);

            stbtt_PackFontRangesPackRects(&pc, rects, n);

            error = stbtt_PackFontRangesRenderIntoRects(&pc, &font_info, ranges, len(ranges), rects);

            //STBTT_free(rects, &pc->user_allocator_context);
        }

        if(!error)
        {
            printf("error loading font\n");
        }
        stbtt_PackEnd(&pc);

        stbtt_GetFontVMetrics(&font_info, &font_ascent, &font_descent, &font_line_gap);

        // char c = 'j';
        // uint16 x0 = font_pack_data[c-32].x0;
        // uint16 y0 = font_pack_data[c-32].y0;
        // uint16 x1 = font_pack_data[c-32].x1;
        // uint16 y1 = font_pack_data[c-32].y1;

        // printf("%c, %d, %d, %d, %d\n", c, x0, y0, x1, y1);

        // for(int i = 0; i < font_bitmap_width*font_bitmap_height; i++)
        // {
        //     putchar(" .:ioVM@"[font_bitmap[i]>>5]);
        //     if(i%font_bitmap_width == font_bitmap_width-1) putchar('\n');
        // }

        /*
          char c;
          uint16 x0 = font_pack_data[c-32].x0;
          uint16 y0 = font_pack_data[c-32].y0;
          uint16 x1 = font_pack_data[c-32].x1;
          uint16 y1 = font_pack_data[c-32].y1;
        */

        //create font-texture
        glGenTextures(1, &ftex);
        glBindTexture(GL_TEXTURE_2D, ftex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, font_bitmap_width, font_bitmap_height, 0, GL_RED, GL_UNSIGNED_BYTE, font_bitmap);
        //GL_ALPHA was removed for some reason
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    }

    GLuint program;
    GLuint transform_uniform;
    {
        void * temp_memory = free_memory;
        char * vertex_shader_source = (char *) temp_memory;
        readTextFile(vertex_shader_source, "shader.vert");
        GLuint vertex_shader = initShader(vertex_shader_source, GL_VERTEX_SHADER);

        char * fragment_shader_source = (char *) temp_memory;
        int fragment_shader_source_size = readTextFile(vertex_shader_source, "shader.frag");
        temp_memory = (void *) ((char *) temp_memory+fragment_shader_source_size);
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

        transform_uniform = glGetUniformLocation(program, "t");

        glDetachShader(program, vertex_shader);
        glDetachShader(program, fragment_shader);
        glDeleteShader(vertex_shader);
        glDeleteShader(fragment_shader);
    }

    GLuint ui_program;
    GLuint ftex_uniform;
    {
        void * temp_memory = free_memory;
        char * vertex_shader_source = (char *) temp_memory;
        readTextFile(vertex_shader_source, "ui_shader.vert");
        GLuint vertex_shader = initShader(vertex_shader_source, GL_VERTEX_SHADER);

        char * fragment_shader_source = (char *) temp_memory;
        int fragment_shader_source_size = readTextFile(vertex_shader_source, "ui_shader.frag");
        temp_memory = (void *) ((char *) temp_memory+fragment_shader_source_size);
        GLuint fragment_shader = initShader(fragment_shader_source, GL_FRAGMENT_SHADER);

        ui_program = glCreateProgram();
        glAttachShader(ui_program, vertex_shader);
        glAttachShader(ui_program, fragment_shader);

        glLinkProgram(ui_program);

        int error;
        glGetProgramiv(ui_program, GL_LINK_STATUS, &error);
        if(error == 0)
        {
            char * info_log = (char *) temp_memory;
            int info_log_size;
            glGetProgramiv(ui_program, 0, &info_log_size);
            glGetProgramInfoLog(ui_program, info_log_size, 0, info_log);
            printf("%s\n", info_log);
            exit(EXIT_FAILURE);
        }

        ftex_uniform = glGetUniformLocation(ui_program, "tex");
        ui_uv0 = glGetUniformLocation(ui_program, "uv0");
        ui_uv1 = glGetUniformLocation(ui_program, "uv1");

        ui_c0 = glGetUniformLocation(ui_program, "c0");
        ui_c1 = glGetUniformLocation(ui_program, "c1");

        ui_color = glGetUniformLocation(ui_program, "color");
        ui_mode = glGetUniformLocation(ui_program, "mode");

        glDetachShader(ui_program, vertex_shader);
        glDetachShader(ui_program, fragment_shader);
        glDeleteShader(vertex_shader);
        glDeleteShader(fragment_shader);
    }

    {
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        glDepthFunc(GL_GEQUAL);
        glDepthRange(0.0f, 1.0f);
        glEnable(GL_DEPTH_CLAMP);

        glFrontFace(GL_CCW);
        glCullFace(GL_BACK);
        glEnable(GL_CULL_FACE);

        glClearColor(0.55, 0.082, 0.082, 1.0);
        glClearDepth(0.0);

        SDL_GL_SetSwapInterval(0); //vsync

        glEnable (GL_BLEND);
        glBlendFunc (GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
    }

    {//necessary for opengl >3.3
        GLuint vao;
        glGenVertexArrays(1, &vao);
        glBindVertexArray(vao);
        glEnableVertexAttribArray(attrib_pos);
    }

    SDL_Event event;

    #define max_models 128 //TODO: allow  "infinite" models
    vi_buffer * models = (vi_buffer *) free_memory;
    free_memory = (byte *) ((vi_buffer *) free_memory + max_models);
    uint n_models = 0;

    #define model_index_table_length 96 //must be less than max_models
    //elements past model_index_table_length will be used to handle collisions of (id%model_index_table_length)
    id_to_index * model_index_table = (id_to_index *) free_memory; //must be zeroed
    memset(model_index_table, 0, max_models*sizeof(id_to_index));
    free_memory = (byte *) ((id_to_index *) free_memory + max_models);
    uint n_overflow = 0;

    //add model to list and index table, units are in inches
    {
        //test
        // float vertex_buffer[] = {
        //     +16.5/2, +0.5, +0.5, //0
        //     +16.5/2, +0.5, -0.5, //1
        //     +16.5/2, -0.5, +0.5, //2
        //     +16.5/2, -0.5, -0.5, //3
        //     -16.5/2, +0.5, +0.5, //4
        //     -16.5/2, +0.5, -0.5, //5
        //     -16.5/2, -0.5, +0.5, //6
        //     -16.5/2, -0.5, -0.5, //7
        // };
        //
        // uint16 rectangular_prism_buffer[] = {
        //     0, 1, 2, 1, 3, 2,
        //     0, 4, 1, 1, 4, 5,
        //     0, 2, 4, 2, 6, 4,
        //     7, 5, 6, 6, 5, 4,
        //     7, 6, 3, 6, 2, 3,
        //     7, 3, 5, 5, 3, 1,
        // };
        //
        // float drive_base_vertices_buffer[] = {
        //     +17.0/2, +0.5, +17.0/2, //0
        //     +17.0/2, +0.5, -17.0/2, //1
        //     +17.0/2, -0.5, +17.0/2, //2
        //     +17.0/2, -0.5, -17.0/2, //3
        //     -17.0/2, +0.5, +17.0/2, //4
        //     -17.0/2, +0.5, -17.0/2, //5
        //     -17.0/2, -0.5, +17.0/2, //6
        //     -17.0/2, -0.5, -17.0/2, //7
        // };
    //end

        models[n_models] = createVertexAndIndexBuffer(sizeof(vertex_buffer), vertex_buffer, sizeof(rectangular_prism_buffer), rectangular_prism_buffer); //test
        models[n_models+1] = createVertexAndIndexBuffer(sizeof(drive_base_vertices_buffer), drive_base_vertices_buffer, sizeof(rectangular_prism_buffer), rectangular_prism_buffer);
        id_to_index id;
        id.id = (typeof(id.id)) {'c','u','b','e',0,0,0,0,};
        id.index = n_models;
        uint current_spot = id.id.id%model_index_table_length;
        //search for empty spot in bucket
        if(model_index_table[current_spot].id.id != 0)
        {
            for(; model_index_table[current_spot].next != 0; current_spot = model_index_table[current_spot].next){}
            current_spot = n_overflow+model_index_table_length;
        }
        model_index_table[current_spot] = id;
        n_models++;
    }

    GLuint quad_vb;
    {
        float vertex_buffer[] = {
            0.0, 0.0,
            0.0, 1.0,
            1.0, 0.0,
            1.0, 1.0,
        };

        glGenBuffers(1, &quad_vb);
        glBindBuffer(GL_ARRAY_BUFFER, quad_vb);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertex_buffer), vertex_buffer, GL_STATIC_DRAW);
    }

    float aspect_ratio = (float) window_width/window_height;
    float n = 19.0;//0.5
    float fov = pi/180.0*120.0;
    float f = 20.0;//10.0;
    m4x4f perspective = {
        tan(pi/2-fov*0.5), 0.0                             , 0.0        ,  0.0,
        0.0              , (tan(pi/2-fov*0.5))*aspect_ratio, 0.0        ,  0.0,
        0.0              , 0.0                             , n/(f-n), -1.0,
        0.0              , 0.0                             , f*n/(f-n),  0.0,
    };

    m4x4f camera = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    };

    camera = multiplyA(camera, perspective);

    for(int i = 0; i < 16; i++)
    {
        printf("%f, ", camera[i]);
        if(i%4 == 3) printf("\n");
    }

    printf("\n");

    render_list = (render_command *) free_memory;

    float angle0 = 0.0;
    float angle1 = 0.0;

    int frame_counter = 0;
    simulatorStartMain();
    for ever
    {
        prev_left_click = left_click;
        while (SDL_PollEvent(&event))
        {
            switch (event.type)
            {
                case SDL_MOUSEBUTTONDOWN:
                {
                    if (event.button.button == SDL_BUTTON_LEFT) left_click = 1;
                    break;
                }
                case SDL_MOUSEBUTTONUP:
                {
                    if (event.button.button == SDL_BUTTON_LEFT) left_click = 0;
                    break;
                }
                case SDL_WINDOWEVENT:
                {
                    switch(event.window.event)
                    {
                        case SDL_WINDOWEVENT_RESIZED:
                            SDL_GL_GetDrawableSize(window, &window_width, &window_height);
                            wx_scale = 2.0/window_width;
                            wy_scale = 2.0/window_height;
                            {
                                float aspect_ratio = (float) window_width/window_height;
                                float n = 0.5;
                                float fov = pi/180.0*120.0;
                                float f = 10.0;
                                perspective = (m4x4f) {
                                    tan(pi/2-fov*0.5), 0.0                             , 0.0        ,  0.0,
                                    0.0              , (tan(pi/2-fov*0.5))*aspect_ratio, 0.0        ,  0.0,
                                    0.0              , 0.0                             , n/(f-n), -1.0,
                                    0.0              , 0.0                             , f*n/(f-n),  0.0,
                                };
                                camera = (m4x4f) {
                                    1.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 1.0, 0.0,
                                    0.0, 1.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 1.0,
                                };

                                camera = multiplyA(camera, perspective);
                            }
                            break;
                    }
                }
                break;
                case SDL_QUIT:
                    return 0;
            }
        }

        // angle0 += 0.0005;//3.1415926535897932384626433832795/4.0;
        // if(angle0 >= 2*3.1416)
        // {
        //     angle0 -= 2*3.1415926535897932384626433832795;
        // }
        // float sine0 = sin(angle0);
        // float cosine0 = cos(angle0);
		//
        // angle1 += 0.00005;
        // if(angle1 >= 2*3.1416)
        // {
        //     angle1 -= 2*3.1415926535897932384626433832795;
        // }
        // float sine1 = sin(angle1);
        // float cosine1 = cos(angle1);
		//
        // for(int i = 0; i < 100; i++)
        // {
        //     render_list[n_to_render].model = 0;
        //     render_list[n_to_render].position = (v3f) {1.0*cos(angle0+sin(angle0)*(100-i)), -1.0*(100-i), sin(angle0+sin(angle0)*(100-i))};
        //     render_list[n_to_render].orientation = (v4f) {cosine1*cosine0, sine1*cosine0, 0.0, -sine0};
        //     n_to_render++;
		//
        //     render_list[n_to_render].model = 0;
        //     render_list[n_to_render].position = (v3f) {-1.0*cos(angle0+sin(angle0)*(100-i)), -1.0*(100-i), -sin(angle0+sin(angle0)*(100-i))};
        //     render_list[n_to_render].orientation = (v4f) {cosine1*cosine0, sine1*cosine0, 0.0, sine0};
        //     n_to_render++;
        // }

        glUseProgram(program);
        glViewport(0, 0, window_width, window_height);
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

        glEnable(GL_DEPTH_TEST);
        glEnable(GL_CULL_FACE);

        //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); //wireframe

        for(uint i = 0; i < n_to_render; i++)
        {
            m4x4f transform = quaternionTo4x4Matrix(render_list[i].orientation);

            transform.rows[3] = *((v4f*) &render_list[i].position);
            transform[15] = 1.0;

            transform = multiplyA(transform, camera);

            glUniformMatrix4fv(transform_uniform, 1, 1, (GLfloat *) &transform);
            bindVertexAndIndexBuffers(models[render_list[i].model].vb, models[render_list[i].model].ib);
            glDrawElements(GL_TRIANGLES, models[render_list[i].model].n, GL_UNSIGNED_SHORT, 0);
        }
        n_to_render = 0;

        glUseProgram(ui_program);

        glDisable(GL_DEPTH_TEST);
        glDisable(GL_CULL_FACE);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, ftex);
	glUniform1i(ftex_uniform, 0);

        glBindBuffer(GL_ARRAY_BUFFER, quad_vb);
        glEnableVertexAttribArray(attrib_pos);
        glVertexAttribPointer(attrib_pos, 2, GL_FLOAT, GL_FALSE, 8, 0);

        //drawui here
        float x_pos = -1.0;
        if(doButtonNW("hello", x_pos, 1.0, 4, 2))
        {
            angle0 += 0.1;
        }
        x_pos += getTextWidthInWindowUnits("hello")+(2*4)*wx_scale;
        if(doButtonNW("goodbye", x_pos, 1.0, 4, 2))
        {
            angle0 -= 0.1;
        }

        if((frame_counter++)%20 == 0) JNI_main(0, (jobject){});

        simulateAndRender();
        leftTrajStatus = leftTraj.positionCalc(leftTrajStatus.w);
        rightTrajStatus = rightTraj.positionCalc(rightTrajStatus.w);
        motionProfileIOTime += motionProfileIOdt;
        fprintf(MotionProfileIO,"%.2f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f,%.4f\n", motionProfileIOTime, leftTrajStatus.x,
        leftTrajStatus.y, leftTrajStatus.z, rightTrajStatus.x, rightTrajStatus.y, rightTrajStatus.z, leftTraj.calculate(leftTrajStatus.w, leftTrajStatus.z),rightTraj.calculate(rightTrajStatus.w, rightTrajStatus.z));
        if(motionProfileIOTime >= 1)
        {
            printf("Safe to close\n");
        }
        SDL_GL_SwapWindow(window);
    }
	fclose(MotionProfileIO);
    SDL_GL_DeleteContext(context);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
