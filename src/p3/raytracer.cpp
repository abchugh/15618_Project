/**
 * @file raytacer.cpp
 * @brief Raytracer class
 *
 * Implement these functions for project 4.
 *
 * @author H. Q. Bovik (hqbovik)
 * @bug Unimplemented
 */

#include "raytracer.hpp"
#include "scene/scene.hpp"

#include <SDL_timer.h>
#include <iostream>
#include <algorithm>
#include <random>
#include <time.h>

#ifdef OPENMP // just a defense in case OpenMP is not installed.

#include <omp.h>

#endif
namespace _462 {

// max number of threads OpenMP can use. Change this if you like.
#define MAX_THREADS 8

struct Options;

static const unsigned STEP_SIZE = 8;

Raytracer::Raytracer()
    : scene(0), width(0), height(0) { }

// random real_t in [0, 1)
static inline real_t random()
{
    return real_t(rand())/RAND_MAX;
}

Raytracer::~Raytracer() { }

/**
 * Initializes the raytracer for the given scene. Overrides any previous
 * initializations. May be invoked before a previous raytrace completes.
 * @param scene The scene to raytrace.
 * @param width The width of the image being raytraced.
 * @param height The height of the image being raytraced.
 * @return true on success, false on error. The raytrace will abort if
 *  false is returned.
 */
bool Raytracer::initialize(Scene* scene, int num_samples, int num_glossy_reflection_samples,
						   size_t width, size_t height)
{
    /*
     * omp_set_num_threads sets the maximum number of threads OpenMP will
     * use at once.
     */
#ifdef OPENMP
    omp_set_num_threads(MAX_THREADS);
#endif
    this->scene = scene;
    this->num_samples = num_samples;
    scene->SetGlossyReflectionSamples(num_glossy_reflection_samples);
    this->width = width;
    this->height = height;

    current_row = 0;

    Ray::init(scene->camera);
	//initialisation done in main
    //scene->initialize();


    return true;
}

/**
 * Performs a raytrace on the given pixel on the current scene.
 * The pixel is relative to the bottom-left corner of the image.
 * @param scene The scene to trace.
 * @param x The x-coordinate of the pixel to trace.
 * @param y The y-coordinate of the pixel to trace.
 * @param width The width of the screen in pixels.
 * @param height The height of the screen in pixels.
 * @return The color of that pixel in the final image.
 */
Color3 Raytracer::trace_pixel(const Scene* scene,
			      size_t x,
			      size_t y,
			      size_t width,
			      size_t height)
{
    assert(x < width);
    assert(y < height);

    real_t dx = real_t(1)/width;
    real_t dy = real_t(1)/height;

    Color3 res = Color3::Black();
    unsigned int iter;

    for (iter = 0; iter < num_samples; iter++)
    {
        // pick a point within the pixel boundaries to fire our
        // ray through.
        real_t i = real_t(2)*(real_t(x)+random())*dx - real_t(1);
        real_t j = real_t(2)*(real_t(y)+random())*dy - real_t(1);

        Ray r = Ray(scene->camera.get_position(), Ray::get_pixel_dir(i, j));
	std::vector<real_t> refractiveStack;
	refractiveStack.push_back(scene->refractive_index);

	hitRecord h;
	h.t = 1e30;
	res += scene->getColor(r, refractiveStack, h);
    }
    return res*(real_t(1)/num_samples);
}

void Raytracer::build_frustum(Frustum& frustum, real_t xmin, real_t xmax,
		   real_t ymin, real_t ymax) {
    // Primary packet is shot from eye position.
    Vector3 eye = scene->camera.get_position();
    Vector3 gaze = normalize(scene->camera.get_direction());

    // Point to inside.
    frustum.planes[FRONT].norm = -gaze;
    frustum.planes[BACK].norm = gaze;

    frustum.planes[FRONT].point = eye + gaze * 
	scene->camera.get_near_clip();
    frustum.planes[BACK].point = eye + gaze * 
	scene->camera.get_far_clip();

    Vector3 low_left = Ray::get_pixel_dir(xmin, ymin) - eye;
    Vector3 low_right = Ray::get_pixel_dir(xmax, ymin) - eye;
    Vector3 up_left = Ray::get_pixel_dir(xmin, ymax) - eye;
    Vector3 up_right = Ray::get_pixel_dir(xmax, ymax) - eye;

    frustum.planes[TOP].norm = cross(up_left, up_right);
    frustum.planes[BOTTOM].norm = cross(low_right, low_left);
    frustum.planes[LEFT].norm = cross(low_left, up_left);
    frustum.planes[RIGHT].norm = cross(up_right, low_right);

    frustum.planes[TOP].point = frustum.planes[BOTTOM].point = 
	frustum.planes[LEFT].point = frustum.planes[RIGHT].point = eye;
}

Packet Raytracer::build_packet(size_t x, size_t y, size_t width, size_t height) {
    real_t dx = real_t(1)/width;
    real_t dy = real_t(1)/height;
    Packet packet(packet_width_ray * packet_width_ray);
    real_t x_min = BIG_NUMBER;
    real_t x_max = -BIG_NUMBER;
    real_t y_min = BIG_NUMBER;
    real_t y_max = -BIG_NUMBER;
    size_t count = 0;

    for (size_t j = 0; j < packet_width_pixel; j++) {
	for (size_t i = 0; i < packet_width_pixel; i++) {
	    size_t cur_x = x + i;
	    size_t cur_y = y + j;

	    if (cur_x >= width || cur_y >= height)
		continue;

	    for (size_t iter = 0; iter < num_samples; iter++) {
		// pick a point within the pixel boundaries to fire our
		// ray through.
		real_t rand_i = real_t(2)*(real_t(cur_x)+random())*dx - real_t(1);
		real_t rand_j = real_t(2)*(real_t(cur_y)+random())*dy - real_t(1);

		Ray r = Ray(scene->camera.get_position(), Ray::get_pixel_dir(rand_i, rand_j));
		packet.rays[count++] = r;

		// Maybe it's better to use the pixel corners
		x_min = std::min(x_min, real_t(rand_i));
		x_max = std::max(x_max, real_t(rand_i));
		y_min = std::min(y_min, real_t(rand_j));
		y_max = std::max(y_max, real_t(rand_j));
	    }
	}
    }

    build_frustum(packet.frustum, x_min, x_max, y_min, y_max);
    packet.size = packet_width_ray * packet_width_ray;

    return packet;
}

void Raytracer::trace_small_packet(unsigned char* buffer,
				   size_t width,
				   size_t height) {
    size_t work_num_x = (width + pixel_width - 1) / pixel_width;
    size_t work_num_y = (height + pixel_width - 1) / pixel_width;

    size_t packet_ray_size = packet_width_ray * packet_width_ray;
    size_t num_packet = (num_samples + packet_ray_size) / packet_ray_size;
    
#pragma omp parallel for schedule(dynamic)
    for (size_t work_count = 0; work_count < work_num_x * work_num_y; work_count++) {
	size_t cur_work_y = work_count / work_num_x;
	size_t cur_work_x = work_count - cur_work_y * work_num_x;

	for (size_t y = 0; y < pixel_width; y++) {
	    for (size_t x = 0; x < pixel_width; x++) {
		size_t p_x = cur_work_x * pixel_width + x;
		size_t p_y = cur_work_y * pixel_width + y;
		if ((p_x + x) >= width || (p_y + y) >= height)
		    continue;

		Color3 cur_color = Color3::Black();
		Color3 packet_color[packet_width_pixel * packet_width_pixel * num_samples]; // init?
		
		for (size_t i = 0; i < num_packet; i++) {
		    Packet packet = build_packet(p_x, p_y, width, height);

		    // TODO: Get color

		    for (size_t count = 0; count < num_samples; count++) 
			cur_color += packet_color[count];
		}

		cur_color = cur_color*(real_t(1)/num_samples);
		cur_color.to_array(&buffer[4 * ((p_y + y) * width + p_x + x)]);
	    }
	}
    }
}

void Raytracer::trace_large_packet(unsigned char* buffer,
				   size_t width,
				   size_t height) {

    size_t work_num_x = (width + pixel_width - 1) / pixel_width;
    size_t work_num_y = (height + pixel_width - 1) / pixel_width;
    
    // Work should be balanced automatically.
#pragma omp parallel for schedule(dynamic)
    for (size_t work_count = 0; work_count < work_num_x * work_num_y; work_count++) {
	size_t cur_work_y = work_count / work_num_x;
	size_t cur_work_x = work_count - cur_work_y * work_num_x;

	// All numbers should have been rounded
	for (size_t cur_packet_y = 0; cur_packet_y < pixel_width / packet_width_pixel; cur_packet_y++) {
	    for (size_t cur_packet_x = 0; cur_packet_x < pixel_width / packet_width_pixel; cur_packet_x++) {
		size_t p_x = cur_packet_x * packet_width_pixel + cur_work_x * pixel_width;
		size_t p_y = cur_packet_y * packet_width_pixel + cur_work_y * pixel_width;
		
		Packet packet = build_packet(p_x, p_y, width, height);
		
		// TODO: Get color here. (func in scene)
		Color3 packet_color[packet_width_pixel * packet_width_pixel * num_samples];
		
		for (size_t y = 0; y < packet_width_pixel; y++) {
		    for (size_t x = 0; x < packet_width_pixel; x++) {
			Color3 cur_color = Color3::Black();
			if ((p_x + x) >= width || (p_y + y) >= height)
			    continue;

			for (size_t count = 0; count < num_samples; count++) 
			    cur_color += packet_color[y * packet_width_pixel * num_samples +
						      x * num_samples + count];
			cur_color = cur_color*(real_t(1)/num_samples);
			cur_color.to_array(&buffer[4 * ((p_y + y) * width + p_x + x)]);
		    }
		}
	    }
	}
    }
}


/**
 * Raytraces some portion of the scene. Should raytrace for about
 * max_time duration and then return, even if the raytrace is not copmlete.
 * The results should be placed in the given buffer.
 * @param buffer The buffer into which to place the color data. It is
 *  32-bit RGBA (4 bytes per pixel), in row-major order.
 * @param max_time, If non-null, the maximum suggested time this
 *  function raytrace before returning, in seconds. If null, the raytrace
 *  should run to completion.
 * @return true if the raytrace is complete, false if there is more
 *  work to be done.
 */
    bool Raytracer::raytrace(unsigned char* buffer, real_t* max_time, bool packet_tracing)
{
    static time_t startTime = SDL_GetTicks();
    if(current_row==0)
	startTime = SDL_GetTicks();
	
    // TODO Add any modifications to this algorithm, if needed.
	
    //static const size_t PRINT_INTERVAL = 64;

    // the time in milliseconds that we should stop
    unsigned int end_time = 0;
    bool is_done = false;

    if (max_time)
    {
        // convert duration to milliseconds
        unsigned int duration = (unsigned int) (*max_time * 1000);
        end_time = SDL_GetTicks() + duration;
    }

    if (!packet_tracing) {
	// until time is up, run the raytrace. we render an entire group of
	// rows at once for simplicity and efficiency.
	for (; !max_time || end_time > SDL_GetTicks(); current_row += STEP_SIZE)
	    {
		// we're done if we finish the last row
		is_done = current_row >= height;
		// break if we finish
		if (is_done) break;

		int loop_upper = std::min(current_row + STEP_SIZE, height);

		// This tells OpenMP that this loop can be parallelized.
#pragma omp parallel for
		for (int c_row = current_row; c_row < loop_upper; c_row++)
		    {
			/*
			 * This defines a critical region of code that should be
			 * executed sequentially.
			 */
			/*#pragma omp critical
			  {
			  if (c_row % PRINT_INTERVAL == 0)
			  printf("Raytracing (Row %d)\n", c_row);
			  }*/

			for (size_t x = 0; x < width; x++)
			    {
				// trace a pixel
				Color3 color = trace_pixel(scene, x, c_row, width, height);
				// write the result to the buffer, always use 1.0 as the alpha
				color.to_array(&buffer[4 * (c_row * width + x)]);
			    }
		    }
	    }
    }
    else {
	if (num_samples >= packet_width_ray * packet_width_ray) {
	    packet_width_pixel = 1;
	    trace_small_packet(buffer, width, height);
	}
	else {
	    // Recompute the unit amount of work
	    size_t num_p = packet_width_ray * packet_width_ray / num_samples;
	    packet_width_pixel = std::floor(std::sqrt(num_p));
	    num_samples = packet_width_ray * packet_width_ray / (packet_width_pixel * packet_width_pixel);
	    pixel_width = pixel_width / packet_width_pixel * packet_width_pixel;

	    trace_large_packet(buffer, width, height);
	}

    }

    time_t endTime = SDL_GetTicks();
    if (is_done) printf("Done raytracing! %ld\n", endTime-startTime);

    return is_done;
}

} /* _462 */
