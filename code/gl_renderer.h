/*
gl_renderer.h - Storage for our models that get rendered, TODO Model loading
*/
#ifndef GL_RENDERER_OURS
#define GL_RENDERER_OURS
//To use: Add objects here, need both the vertex buffer and index buffer, also add the object to the enum for clarity
	float vertex_buffer[] = {
		+16.5/2, +0.5, +0.5, //0
		+16.5/2, +0.5, -0.5, //1
		+16.5/2, -0.5, +0.5, //2
		+16.5/2, -0.5, -0.5, //3
		-16.5/2, +0.5, +0.5, //4
		-16.5/2, +0.5, -0.5, //5
		-16.5/2, -0.5, +0.5, //6
		-16.5/2, -0.5, -0.5, //7
	};

	uint16 rectangular_prism_buffer[] = {
		0, 1, 2, 1, 3, 2,
		0, 4, 1, 1, 4, 5,
		0, 2, 4, 2, 6, 4,
		7, 5, 6, 6, 5, 4,
		7, 6, 3, 6, 2, 3,
		7, 3, 5, 5, 3, 1,
	};

	float drive_base_vertices_buffer[] = {
		+17.0/2, +0.5, +17.0/2, //0
		+17.0/2, +0.5, -17.0/2, //1
		+17.0/2, -0.5, +17.0/2, //2
		+17.0/2, -0.5, -17.0/2, //3
		-17.0/2, +0.5, +17.0/2, //4
		-17.0/2, +0.5, -17.0/2, //5
		-17.0/2, -0.5, +17.0/2, //6
		-17.0/2, -0.5, -17.0/2, //7
	};
	enum
	{
		arm_bar = 0,
		drive_base
	};



#endif
