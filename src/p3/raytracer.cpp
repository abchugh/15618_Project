/**
* @file raytacer.cpp
* @brief Raytracer class
*
* Implement these functions for project 4.
*
* @author H. Q. Bovik (hqbovik)
* @bug Unimplemented
*/

#include "application/application.hpp"
#include "raytracer.hpp"
#include "scene/scene.hpp"
#include "material/material.hpp"
#include "integrator/direct.hpp"
#include "integrator/whitted.hpp"
#include "integrator/path.hpp"
#include "light/infinite.hpp"

#include <SDL_timer.h>
#include <iostream>
#include <algorithm>
#include <random>
#include <time.h>
#include <vector>
#include <omp.h>

using namespace std;

namespace _462 {
	
    extern time_t ta[MAX_THREADS], tb[MAX_THREADS],ts[MAX_THREADS], tt[MAX_THREADS], tu[MAX_THREADS];
    time_t tc[MAX_THREADS],td[MAX_THREADS],te[MAX_THREADS],tf[MAX_THREADS];
    static time_t sum(time_t arr[])
    {
        time_t sum = 0;
        for(int i=0;i<MAX_THREADS;i++)
            sum+=arr[i];
        return sum;
    }
    static void reset()
    {
        for(int i=0;i<MAX_THREADS;i++)
            ta[i] = tb[i] = tc[i] = td[i] = te[i] = tf[i] = 0;
    }
    // max number of threads OpenMP can use. Change this if you like.
#define MAX_THREADS 8

    struct Options;

    static const unsigned STEP_SIZE = 8;

    static inline int nextPow2(int n)
    {
        n--;
        n |= n >> 1;
        n |= n >> 2;
        n |= n >> 4;
        n |= n >> 8;
        n |= n >> 16;
        n++;

        return n;
    }

    static inline void roundPacket(uint32_t p_r_width, uint32_t num_sample,
		     uint32_t &p_p_x, uint32_t &p_p_y) {
	uint32_t num_pixel = p_r_width * p_r_width / num_sample;
	uint32_t x = num_pixel, y = 1;

	while ((x & 0x1) == 0 && 2 * y < x) {
	    x >>= 1;
	    y <<= 1;
	}

	p_p_x = x;
	p_p_y = y;
    }

    Raytracer::Raytracer()
        : scene(0), width(0), height(0) { }

    Raytracer::~Raytracer() {
	delete sampler_ptr;
	delete film_ptr;

	sampler_ptr = NULL;
	film_ptr = NULL;
    }

    /**
    * Initializes the raytracer for the given scene. Overrides any previous
    * initializations. May be invoked before a previous raytrace completes.
    * @param scene The scene to raytrace.
    * @param width The width of the image being raytraced.
    * @param height The height of the image being raytraced.
    * @return true on success, false on error. The raytrace will abort if
    *  false is returned.
    */
    bool Raytracer::initialize(Scene* scene, Options *opt_ptr,
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
        this->num_samples = opt_ptr->num_samples;
		this->opt_ptr = opt_ptr;
        scene->SetGlossyReflectionSamples(opt_ptr->num_glossy_reflection_samples);

        current_row = 0;

        Ray::init(scene->camera);
        //initialisation done in main
        //scene->initialize();

        // Set parameters for packet tracing
        // Every openmp thread grasps a pixel_width x pixel_width
        // grid. Every packet contains 
        // packet_width_ray x packet_width_ray rays.
        this->pixel_width = opt_ptr->pixel_width;
        this->packet_width_ray = opt_ptr->packet_width_ray;

        this->num_threads = opt_ptr->num_threads;

	roundPacket(this->packet_width_ray, this->num_samples, this->packet_width_x,
		    this->packet_width_y);

	//Filter *filter_ptr = new GaussianFilter(2.f, 2.f, 2.f);
	Filter *filter_ptr = new MitchellFilter(2.f, 2.f, 1.f / 3, 1.f / 3);
	//Filter *filter_ptr = new SincFilter(4.f, 4.f, 3.f);
	//Filter *filter_ptr = new BoxFilter(0.5f, 0.5f);

	uint32_t filter_x = (uint32_t)(filter_ptr->width_x + 0.5f);
	uint32_t filter_y = (uint32_t)(filter_ptr->width_y + 0.5f);

	uint32_t padding_x = filter_x / this->packet_width_x * 
	    this->packet_width_x + 
	    (filter_x % this->packet_width_x > 0) * this->packet_width_x;
	uint32_t padding_y = filter_y / this->packet_width_y * 
	    this->packet_width_y + 
	    (filter_y % this->packet_width_y > 0) * this->packet_width_y;

	//padding_x = padding_y = 16;

    this->width = width + 2 * padding_x;
	this->height = height + 2 * padding_y;

	printf("w: %d; h: %d\n", width, height);
	printf("ray: %d; #sam: %d; x: %d; y: %d\n", this->packet_width_ray,
	       this->num_samples, this->packet_width_x, this->packet_width_y);

	sampler_ptr = new HaltonSampler(this->width, this->height, this->packet_width_x,
					    this->packet_width_y, this->num_samples);
	
	
	film_ptr = new Film(this->width,
			    this->height,
			    padding_x, width + padding_x,
			    padding_y, height + padding_y);
	film_ptr->setFilter(filter_ptr);

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
            real_t i = real_t(2)*(real_t(x)+ rng.random())*dx - real_t(1);
            real_t j = real_t(2)*(real_t(y)+ rng.random())*dy - real_t(1);

            Ray r = Ray(scene->camera.get_position(), Ray::get_pixel_dir(i, j));
            std::vector<real_t> refractiveStack;
            refractiveStack.push_back(scene->refractive_index);

            res += scene->getColor(r, refractiveStack);
        }
        return res*(real_t(1)/num_samples);
    }

    void Raytracer::build_frustum(Frustum& frustum, real_t xmin, real_t xmax,
        real_t ymin, real_t ymax) {
            // Primary packet is shot from eye position.
            Vector3 eye = scene->camera.get_position();
            Vector3 gaze = scene->camera.get_direction();

            // Point to inside.
            frustum.planes[FRONT].norm = gaze;
            frustum.planes[BACK].norm = -gaze;

            frustum.planes[FRONT].point = eye + gaze * 
                scene->camera.get_near_clip();
            frustum.planes[BACK].point = eye + gaze * 
                scene->camera.get_far_clip();

            Vector3 low_left = get_viewing_ray(xmin, ymin);
            Vector3 low_right = get_viewing_ray(xmax, ymin);
            Vector3 up_left = get_viewing_ray(xmin, ymax);
            Vector3 up_right = get_viewing_ray(xmax, ymax);

            frustum.planes[TOP].norm = cross(up_left, up_right);
            frustum.planes[BOTTOM].norm = cross(low_right, low_left);
            frustum.planes[LEFT].norm = cross(low_left, up_left);
            frustum.planes[RIGHT].norm = cross(up_right, low_right);

            frustum.planes[TOP].point = frustum.planes[BOTTOM].point = 
                frustum.planes[LEFT].point = frustum.planes[RIGHT].point = eye;
            frustum.isValid = true;
    }

    // calculate direction of initial viewing ray from camera
    Vector3 Raytracer::get_viewing_ray(size_t x, size_t y) {
        // normalized camera direction
        Vector3 gaze = scene->camera.get_direction();
        // normalized camera up direction
        Vector3 up = scene->camera.get_up();
        // normalized camera right direction
        Vector3 right = cross(gaze, up);
        // camera field of view
        float fov = scene->camera.get_fov_radians();
        // t, b, r, and l are the border disances of the image plane
        real_t t = tan((width / height) * fov / 2.0);
        real_t r = (t * width) / height;
        real_t b = -1.0 * t;
        real_t l = -1.0 * r;
        // the pixel's horizontal coordinate on image plane
        real_t u = l + (r - l) * (x + 0.5) / width;
        // the pixel's vertical coordinate on image plane
        real_t v = b + (t - b) * (y + 0.5) / height;
        // Shirley uses the near plane for the below calculation; we'll just use 1
        Vector3 view_ray = gaze + (u * right) + (v * up);

        return view_ray; // viewing ray direction
    }

    void Raytracer::build_packet(size_t x, size_t y, size_t width, size_t height, Packet& packet) {
        real_t dx = real_t(1)/width;
        real_t dy = real_t(1)/height;
        size_t x_min;
        size_t x_max;
        size_t y_min;
        size_t y_max;
        size_t count = 0;

        size_t iterations = std::min(packet.size, num_samples);

        for (size_t j = 0; j < packet_width_y; j++) {
            for (size_t i = 0; i < packet_width_x; i++) {
                size_t cur_x = x + i;
                size_t cur_y = y + j;

                if (cur_x >= width || cur_y >= height)
                    continue;

                for (size_t iter = 0; iter < iterations; iter++) {
                    // pick a point within the pixel boundaries to fire our
                    // ray through.
                    real_t rand_i = real_t(2)*(real_t(cur_x)+ rng.random())*dx - real_t(1);
                    real_t rand_j = real_t(2)*(real_t(cur_y)+ rng.random())*dy - real_t(1);

                    Ray r = Ray(scene->camera.get_position(), Ray::get_pixel_dir(rand_i, rand_j));

                    // TODO: a better way?
                    packet.e_x[count] = r.e[0];
                    packet.e_y[count] = r.e[1];
                    packet.e_z[count] = r.e[2];
                    packet.d_x[count] = r.d[0];
                    packet.d_y[count] = r.d[1];
                    packet.d_z[count] = r.d[2];

                    packet.rays[count++] = r;
                }
            }
        }
        x_min = x;
        x_max = x + packet_width_x;
        y_min = y;
        y_max = y + packet_width_y;

        build_frustum(packet.frustum, x_min, x_max, y_min, y_max);

        packet.size = packet_width_ray * packet_width_ray;
    }

    void Raytracer::trace_small_packet(unsigned char* buffer,
        size_t width,
        size_t height) {
            size_t work_num_x = (width + pixel_width - 1) / pixel_width;
            size_t work_num_y = (height + pixel_width - 1) / pixel_width;

            size_t packet_ray_size = packet_width_ray * packet_width_ray;
            size_t num_packet = (num_samples + packet_ray_size) / packet_ray_size;

#pragma omp parallel for schedule(dynamic) num_threads(num_threads)
            for (int work_count = 0; work_count < work_num_x * work_num_y; work_count++) {
                size_t cur_work_y = work_count / work_num_x;
                size_t cur_work_x = work_count - cur_work_y * work_num_x;

                for (size_t y = 0; y < pixel_width; y++) {
                    for (size_t x = 0; x < pixel_width; x++) {
                        size_t p_x = cur_work_x * pixel_width + x;
                        size_t p_y = cur_work_y * pixel_width + y;
                        if ((p_x) >= width || (p_y) >= height)
                            continue;

                        Color3 cur_color = Color3::Black();
                        Color3* packet_color = new Color3[packet_ray_size];

                        // Shoot packet one by one to the same pixel
                        for (size_t i = 0; i < num_packet; i++) {
                            Packet packet(packet_width_ray * packet_width_ray);
                            build_packet(p_x, p_y, width, height, packet);

                            // Get color
			    std::vector< std::vector<real_t> > refractiveStack;
                            for(int i=0;i<packet.size;i++)
                            {
                                std::vector<real_t> rstack;
                                rstack.push_back(scene->refractive_index);
                                refractiveStack.push_back(rstack);
                            }

                            scene->getColors(packet, refractiveStack, packet_color);

                            for (size_t count = 0; count < packet_ray_size; count++) 
                                cur_color += packet_color[count];
                        }

                        cur_color = cur_color*(real_t(1)/(num_packet * packet_ray_size));
                        cur_color.to_array(&buffer[4 * ((p_y) * width + p_x)]);
                    }
                }
            }
    }

    void Raytracer::build_packet_sampler(size_t width, size_t height, Packet& packet,
					 Sample *&samples,
					 size_t &p_x, size_t &p_y, Random462 &rng) {
        size_t x_min;
        size_t x_max;
        size_t y_min;
        size_t y_max;

		uint32_t temp_x, temp_y;
		size_t count = 0;

		float dx = float(1)/width;
		float dy = float(1)/height;

		samples = sampler_ptr->getPacketSamples(temp_x, temp_y, rng);

		p_x = temp_x;
		p_y = temp_y;

        for (size_t j = 0; j < packet_width_y; j++) {
            for (size_t i = 0; i < packet_width_x; i++) {

                if (p_x + i >= width || p_y + j >= height)
                    continue;

                for (size_t iter = 0; iter < num_samples; iter++) {
                    // pick a point within the pixel boundaries to fire our
                    // ray through.
					float *pos_samples = (float*)samples + (j * packet_width_x * num_samples + 
						i * num_samples + iter) * sampler_ptr->get_sample_size();
					float rand_i = 2.f * pos_samples[0] * dx - 1.f;
					float rand_j = 2.f * pos_samples[1] * dy - 1.f;

                    Ray r = Ray(scene->camera.get_position(), Ray::get_pixel_dir(rand_i, rand_j));

                    // TODO: a better way?
                    packet.e_x[count] = r.e[0];
                    packet.e_y[count] = r.e[1];
                    packet.e_z[count] = r.e[2];
                    packet.d_x[count] = r.d[0];
                    packet.d_y[count] = r.d[1];
                    packet.d_z[count] = r.d[2];

                    packet.rays[count++] = r;
                }
            }
        }
        x_min = p_x;
        x_max = p_x + packet_width_x;
        y_min = p_y;
        y_max = p_y + packet_width_y;

        build_frustum(packet.frustum, x_min, x_max, y_min, y_max);

        packet.size = count;
    }

	void Raytracer::trace_packet_integrator(SurfaceIntegrator *integrator_ptr, size_t width, size_t height) {
		uint32_t wanted_packet_num = (width + packet_width_x - 1) / packet_width_x * 
	    ((height + packet_width_y - 1) / packet_width_y);
		// TODO : rng array for multi-threading
		Random462 rng;

		time_t prev_time = -1;
		time_t this_time;

#pragma omp parallel for num_threads(8)
		for (int i = 0; i < wanted_packet_num; i++) {
			int tid = omp_get_thread_num();

			if (tid == 0) {
				this_time = SDL_GetTicks();
				if (this_time - prev_time > 30000) {
					if (prev_time < 0)
						printf("Rendering: ");
					prev_time = this_time;
					printf("%f\%\n", (float)i / wanted_packet_num * 100);
				}
			}

		   // All numbers should have been rounded
		   size_t p_x;
		   size_t p_y;

		   // Clamped along the boundary.
		   Packet packet(packet_width_ray * packet_width_ray);
		   Sample *samples;

		   build_packet_sampler(width, height, packet, samples, p_x, p_y, rng);

		   vector<hitRecord> hs(packet.size);
		   scene->hit(packet, 0.f, BIG_NUMBER, hs, true);
		   
		   for (size_t y = 0; y < packet_width_y; y++) {
			   for (size_t x = 0; x < packet_width_x; x++) {

			   if ((p_x + x) >= width || (p_y + y) >= height)
				   continue;

			   for (size_t count = 0; count < num_samples; count++) {
					   uint32_t offset = y * packet_width_x * num_samples +
					   x * num_samples + count;
					   hs[offset].depth = 0;
					   float* start = (float*)samples + offset * sampler_ptr->get_sample_size();

					   Color3 color = integrator_ptr->li(scene, packet.rays[offset], hs[offset], (Sample*)start, rng);
					   film_ptr->addSample(*(Sample*)start, color);
				   }
			   }
		   }
	    
		}	
	}

    void Raytracer::trace_packet(size_t width, size_t height) {
	uint32_t wanted_packet_num = (width + packet_width_x - 1) / packet_width_x * 
	    ((height + packet_width_y - 1) / packet_width_y);
	Random462 rng;

#pragma omp parallel for num_threads(num_threads)
	for (int i = 0; i < wanted_packet_num; i++) {
           Color3* packet_color = new Color3[packet_width_x * packet_width_y * num_samples];

            // All numbers should have been rounded
	   size_t p_x;
	   size_t p_y;

	   // Clamped along the boundary.
	   Packet packet(packet_width_ray * packet_width_ray);
	   Sample *samples;

	   build_packet_sampler(width, height, packet, samples, p_x, p_y, rng);

	   std::vector< std::vector<real_t> > refractiveStack;
	   for(int i=0;i<packet.size;i++)
	       {
		   std::vector<real_t> rstack;
		   rstack.push_back(scene->refractive_index);
		   refractiveStack.push_back(rstack);
	       }

	   scene->getColors(packet, refractiveStack, packet_color);

	   for (size_t y = 0; y < packet_width_y; y++) {
	       for (size_t x = 0; x < packet_width_x; x++) {

		   if ((p_x + x) >= width || (p_y + y) >= height)
		       continue;

		   for (size_t count = 0; count < num_samples; count++) {
		       uint32_t offset = y * packet_width_x * num_samples +
			   x * num_samples + count;

		       film_ptr->addSample(samples[offset], packet_color[offset]);
		   }
	       }
	   }

	    // Get color here. (func in scene)
	    delete[] packet_color;
	    
	}	
    
    }

    void Raytracer::trace_large_packet(unsigned char* buffer, size_t width, size_t height) {
        size_t work_num_x = (width + pixel_width - 1) / pixel_width;
        size_t work_num_y = (height + pixel_width - 1) / pixel_width;
        reset();
        time_t start = SDL_GetTicks();
        // Work should be balanced automatically.
		//Color3* colorBuffer = new Color3[width*height];
#pragma omp parallel for schedule(dynamic) num_threads(num_threads)
        for (int work_count = 0; work_count < work_num_x * work_num_y; work_count++) {
            size_t cur_work_y = work_count / work_num_x;
            size_t cur_work_x = work_count - cur_work_y * work_num_x;

			
           // Get color here. (func in scene)
           Color3* packet_color = new Color3[packet_width_x * packet_width_y * num_samples];

            // All numbers should have been rounded
            for (size_t cur_packet_y = 0; cur_packet_y < pixel_width / packet_width_y; cur_packet_y++) {
                for (size_t cur_packet_x = 0; cur_packet_x < pixel_width / packet_width_x; cur_packet_x++) {

                    time_t tt = SDL_GetTicks();

                    size_t p_x = cur_packet_x * packet_width_x + cur_work_x * pixel_width;
                    size_t p_y = cur_packet_y * packet_width_y + cur_work_y * pixel_width;

                    // Clamped along the boundary.
                    Packet packet(packet_width_ray * packet_width_ray);
                    build_packet(p_x, p_y, width, height, packet);

                    tc[omp_get_thread_num()] += SDL_GetTicks() -tt;

                    
                    std::vector< std::vector<real_t> > refractiveStack;
                    for(int i=0;i<packet.size;i++)
                    {
                        std::vector<real_t> rstack;
                        rstack.push_back(scene->refractive_index);
                        refractiveStack.push_back(rstack);
                    }

                    td[omp_get_thread_num()] += SDL_GetTicks() -tt;
                    scene->getColors(packet, refractiveStack, packet_color);

                    te[omp_get_thread_num()] += SDL_GetTicks() -tt;

					
                    for (size_t y = 0; y < packet_width_y; y++) {
                        for (size_t x = 0; x < packet_width_x; x++) {

                            Color3 cur_color = Color3::Black();
                            if ((p_x + x) >= width || (p_y + y) >= height)
                                continue;

                            for (size_t count = 0; count < num_samples; count++) 
                                cur_color += packet_color[y * packet_width_x * num_samples +
                                x * num_samples + count];

                            cur_color = cur_color*(real_t(1)/num_samples);
                            cur_color.to_array(&buffer[4 * ((p_y + y) * width + p_x + x)]);
                        }
                    }

                    tf[omp_get_thread_num()] += SDL_GetTicks() -tt;
                }
            }

                    // Get color here. (func in scene)
			delete[] packet_color;
        }

        for(int i=0;i<20;i++)
        {
            if(te[i]==0) break;
            printf("%d ", te[i]);
        }
        printf("\n");
        /*for(int i=0;i<20;i++)
        {
            if(tb[i]==0) break;
            printf("%d ", tb[i]-ta[i]);
        }
        printf("\n");*/

        time_t end = SDL_GetTicks();
        printf("ta tb... = P=%d/%d S=%d/%d Pack=%d Data=%d stack=%d %d %d\n", sum(ta), sum(te)-sum(td), sum(tb), sum(ts), sum(tc), sum(tf) - sum(te), sum(td)-sum(tc), sum(tt), sum(tu));//sum(td)-sum(tc) = tb), sum(te)-sum(td) negligible
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
        //packet_tracing = false;
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
			printf("trace: %d, %d\n", width, height);

			/*
			Matrix3 trans;
			Vector3 z(0.f, 1.f, 0.f);
			Vector3 x, y;
			coordinate_system(z, &x, &y);
			Material *sky_ptr = NULL;
			for (uint32_t i = 0; i < scene->num_materials(); i++) {
				if (!scene->get_materials()[i]->texture_filename.empty())
					sky_ptr = scene->get_materials()[i];
			}
			InfiniteAreaLight inf_light(Matrix3(x, y, z), sky_ptr);
			scene->add_light((Light*)&inf_light);
			*/
			//WhittedIntegrator whitted(4);
			//whitted.initialize_sampler(scene, sampler_ptr);
			//DirectLightingIntegrator dir_int(scene, SAMPLE_ALL, 4);
			//dir_int.initialize_sampler(scene, sampler_ptr);
			PathIntegrator path_int(scene, opt_ptr->sample_depth, opt_ptr->max_depth, opt_ptr->num_per_path);
			path_int.initialize_sampler(scene, sampler_ptr);
			trace_packet_integrator(&path_int, width, height);
			//trace_packet(width, height);
			is_done = true;

			film_ptr->output(buffer);

			//	    exit(0);
			/*

				if (num_samples >= packet_width_ray * packet_width_ray) {
					packet_width_x = packet_width_y = 1;
					printf("pp: %ld; #s: %ld; wp: %ld\n", packet_width_x, num_samples, pixel_width);

					trace_small_packet(buffer, width, height);
					is_done = true;
				}
				else {
					// Recompute the unit amount of work
					// So a packet can perfectly cover a square area of pixels.
			roundPacket(packet_width_ray, num_samples, packet_width_x,
					packet_width_y);
					printf("pp: %d; pr: %d; #s: %d; wp: %d\n", packet_width_x, 
				   packet_width_ray, num_samples, pixel_width);

					trace_large_packet(buffer, width, height);
					is_done = true;
			}*/

        }
        time_t endTime = SDL_GetTicks();

        if (is_done) printf("Done raytracing! %ld\n", endTime-startTime);

        return is_done;
    }

} /* _462 */
