/**
 * @file raytacer.hpp
 * @brief Raytracer class
 *
 * Implement these functions for project 3.
 *
 * @author H. Q. Bovik (hqbovik)
 * @bug Unimplemented
 */

#ifndef _462_RAYTRACER_HPP_
#define _462_RAYTRACER_HPP_

#define MAX_DEPTH 5

#include "sample/stratified.hpp"
#include "sample/halton.hpp"
#include "filter/film.hpp"
#include "filter/box.hpp"
#include "filter/gaussian.hpp"
#include "filter/mitchell.hpp"
#include "filter/sinc.hpp"
#include "math/color.hpp"
#include "math/random462.hpp"
#include "math/vector.hpp"
#include "integrator/surface.hpp"

namespace _462 {

class Scene;
class Ray;
struct Intersection;
struct Options;
struct Packet;
struct Frustum;
struct Options;

class Raytracer
{
public:

    Raytracer();

    ~Raytracer();

	bool initialize(Scene* scene, Options *opt_ptr,
		    size_t width, size_t height);

    bool raytrace(unsigned char* buffer, real_t* max_time, bool packet_tracing);

private:

    Color3 trace_pixel(const Scene* scene,
		       size_t x,
		       size_t y,
		       size_t width,
		       size_t height);
    Vector3 get_viewing_ray(size_t x, size_t y);
    void build_frustum(Frustum& frustum, real_t xmin, real_t xmax,
		       real_t ymin, real_t ymax);

    void build_packet(size_t x, size_t y, size_t width, size_t height, Packet& packet);

    // when multiple packets inside one pixel
    void trace_small_packet(unsigned char* buffer,
			    size_t width,
			    size_t height);

    // when a packet is over multiple pixels
    void trace_large_packet(unsigned char* buffer,
			    size_t width,
			    size_t height);

    void build_packet_sampler(size_t width, size_t height, Packet& packet, Sample *&samples,
		size_t &p_x, size_t &p_y, Random462 &rng);
    void trace_packet(size_t width, size_t height);
	void trace_packet_integrator(SurfaceIntegrator *integrator_ptr, size_t width, size_t height);

    // the scene to trace
    Scene* scene;

    // the dimensions of the image to trace
    size_t width, height;

    // the next row to raytrace
    size_t current_row;
	
    unsigned int num_samples;
    unsigned int num_dof_samples;

    // work size
    uint32_t pixel_width;
    // packet size (ray)
    uint32_t packet_width_ray;
    // packet size (pixel)
    uint32_t packet_width_x, packet_width_y;

    uint32_t num_threads;
    Random462 rng;
    Sampler *sampler_ptr;

	Options *opt_ptr;

    Film *film_ptr;
};

} /* _462 */

#endif /* _462_RAYTRACER_HPP_ */
