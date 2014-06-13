
#ifndef _462_SAMPLER_HPP_
#define _462_SAMPLER_HPP_

#include <vector>
#include <malloc.h>
#include <omp.h>

#include "math/math.hpp"
#include "math/vector.hpp"
#include "scene/bvh.hpp"
#include "math/random462.hpp"

namespace _462 {

inline float balance_heuristic(int nf, float fpdf, int ng, float gpdf) {
	return nf * fpdf / (nf * fpdf + ng * gpdf);
}

inline float power_heuristic(int nf, float fpdf, int ng, float gpdf) {
	float f = nf * fpdf;
	float g = ng * gpdf;
	return f * f / (f * f + g * g);
}

inline Vector3 concentric_sample_disk(float r1, float r2) {
	Vector3 result = Vector3::Zero();
	float r, theta;
    // Map uniform random numbers to $[-1,1]^2$
    float sx = 2 * r1 - 1;
    float sy = 2 * r2 - 1;

    // Map square to $(r,\theta)$

    // Handle degeneracy at the origin
    if (sx == 0.0 && sy == 0.0) {
		result.x = 0.;
		result.y = 0.;

		return result;
    }
    if (sx >= -sy) {
        if (sx > sy) {
            // Handle first region of disk
            r = sx;
            if (sy > 0.0) theta = sy/r;
            else          theta = 8.0f + sy/r;
        }
        else {
            // Handle second region of disk
            r = sy;
            theta = 2.0f - sx/r;
        }
    }
    else {
        if (sx <= sy) {
            // Handle third region of disk
            r = -sx;
            theta = 4.0f - sy/r;
        }
        else {
            // Handle fourth region of disk
            r = -sy;
            theta = 6.0f + sx/r;
        }
    }
    theta *= PI / 4.f;
    result.x = r * cos(theta);
    result.y = r * sin(theta);

	return result;
}

inline Vector3 uniform_sample_sphere(float r1, float r2) {
	float z = 1.f - 2.f * r1;
	float r = std::sqrt(1.f - z * z);
	float phi = 2.f * PI * r2;
	
	return Vector3(r * std::cos(phi), r * std::sin(phi), z);
}

inline Vector3 uniform_sample_cone(float r1, float r2, float cos, 
								   Vector3 x, Vector3 y, Vector3 z) {
	float costheta = (1.f - r1) + r1 * cos;
    float sintheta = sqrt(1.f - costheta*costheta);
    float phi = r2 * 2.f * PI;
    return std::cos(phi) * sintheta * x + std::sin(phi) * sintheta * y + costheta * z;
}

inline Vector2 uniform_sample_triangle(float r1, float r2) {
	Vector2 result;
	float sqrt1 = std::sqrt(r1);
	result.x = 1 - sqrt1;
	result.y = r2 * sqrt1;

	return result;
}

inline float uniform_sample_sphere_pdf() {
	return 1.f * 0.25f / PI;	
}

inline float uniform_sample_cone_pdf(float cos) {
	return 1.f / (2 * PI * (1 - cos));
}

inline Vector3 cos_sampled_hemisphere(float r1, float r2) {
	Vector3 result = concentric_sample_disk(r1, r2);
	result.z = std::sqrt(1.f - result.x * result.x - result.y * result.y);

	return result;
}

void latin_hypercube(float *samples, uint32_t dim, uint32_t num, Random462 &rng);

struct Distribution1D {
	Distribution1D(float *func, uint32_t size);
	~Distribution1D() {
		delete[] cdf;
	}

	float sample_continuous(float r, float *pdf);
	uint32_t sample_discrete(float r, float *pdf);

	float norm;
	float *cdf;
	uint32_t piece_size;
};

struct Distribution2D {
	Distribution2D(float *func, uint32_t size_u, uint32_t size_v);
	~Distribution2D() {
		delete marginal;
		for (uint32_t i = 0; i < conditionals.size(); i++) {
			delete conditionals[i];
		}
	}

	Vector2 sample_continuous(float r1, float r2, float *pdf);
	float pdf(float u, float v);
	uint32_t size_u, size_v;

private:
	
	Distribution1D *marginal;
	std::vector<Distribution1D*> conditionals;
};

struct Sample {
    float x, y;
};

class SampleSet {
public:
    
    SampleSet(uint32_t capacity)
		: samples(NULL), sample_capacity(capacity), current(0), current_one_offset(0),
		current_two_offset(0), max_set_size(800 * 600 * 4) { }
    ~SampleSet() {
		current = 0;
		current_one_offset = 0;
		current_two_offset = 0;
		_aligned_free(samples);
    }

	// not thread-safe
    uint32_t add1D(uint32_t oneD) {
		oneD_num.push_back(oneD);
		uint32_t offset = current_one_offset;
		current_one_offset += oneD_num[oneD_num.size() - 1];

		return offset;
    }

    uint32_t add2D(uint32_t twoD) {
		twoD_num.push_back(twoD);

		uint32_t offset = current_two_offset;
		current_two_offset += twoD_num[twoD_num.size() - 1];

		return offset * 2;
    }

	void add2Dxy() {
		twoD_num.push_back(1);
	}

    void allocateSamples() {
		sample_size = 0;

		for (int i = 0; i < oneD_num.size(); i++) {
			sample_size += oneD_num[i];
		}

		for (int i = 0; i < twoD_num.size(); i++) {
			sample_size += twoD_num[i] * 2;
		}

		alloc_size = (std::min(max_set_size, sample_capacity * sample_size) + sample_size - 1) / sample_size;
		do {
			samples = (float*)memalign(16, sizeof(float) * sample_size * alloc_size);
			if (samples == NULL)
				alloc_size /= 2;
		} while(samples == NULL);
    }

    Sample *addSample(float *oneD, float *twoD) {
		uint32_t des = 0;

	#ifdef _WINDOWS
	#pragma omp critical
		{
			des = current++;
			
		}
	#else
		des = __sync_fetch_and_add(&current, 1);
	#endif
		if (des >= alloc_size)
				des = des % alloc_size;
		assert(des < sample_capacity);

		memcpy(samples + sample_size * des, oneD, sizeof(float) * oneD_num.size());
		memcpy(samples + sample_size * des + oneD_num.size(), 
			   twoD, sizeof(float) * twoD_num.size() * 2);
	
		return (Sample*)(samples + sample_size * des);
    }

    // oneD : one1...
    //        one2...
    Sample *addSamples(float *oneD, float *twoD, uint32_t count) {
	uint32_t des = 0;

	#ifdef _WINDOWS
	#pragma omp critical
		{
			do {
				des = current;
				if (des >= alloc_size)
					des = des % alloc_size;
				current += count;
			} while (des < alloc_size && des + count - 1 >= alloc_size);
		}
	#else
		do {
				des = __sync_fetch_and_add(&current, count);
				if (des >= alloc_size)
					des = des % alloc_size;
			} while (des < alloc_size && des + count - 1 >= alloc_size);
		
	#endif

		assert(des < sample_capacity);

		uint32_t oneD_size = oneD_num.size();
		uint32_t twoD_size = twoD_num.size();

		for (int i = 0; i < count; i++) {
			for (int j = 0; j < oneD_size; j++)
			samples[sample_size * (des + i) + j] = oneD[j * count + i];
			for (int j = 0; j < twoD_size; j++) {
			samples[oneD_size + sample_size * (des + i) + 2 * j] = twoD[j * count * 2 + 2 * i];
			samples[oneD_size + sample_size * (des + i) + 2 * j + 1] = 
				twoD[j * count * 2 + 2 * i + 1];
			}
		}
	
		return (Sample*)(samples + sample_size * des);
    }

    Sample *addEmptySamples(uint32_t count) {
		uint32_t des = -1;

	#ifdef _WINDOWS
	#pragma omp critical
		{
			do {
				des = current;
				if (des >= alloc_size)
					des = des % alloc_size;
				current += count;
			} while (des < alloc_size && des + count - 1 >= alloc_size);
		}
	#else
		do {
				des = __sync_fetch_and_add(&current, count);
				if (des >= alloc_size)
					des = des % alloc_size;
			} while (des < alloc_size && des + count - 1 >= alloc_size);
	#endif

		assert(des >= 0);
		assert(des < sample_capacity);
	
		return (Sample*)(samples + sample_size * des);
    }

    std::vector<uint32_t> oneD_num;
    std::vector<uint32_t> twoD_num;
    uint32_t sample_size;
    float *samples;
    uint32_t sample_capacity;

private:
    uint32_t current;
	uint32_t current_one_offset;
	uint32_t current_two_offset;
	const uint32_t max_set_size;
	uint32_t alloc_size;
};

class Sampler {

public:
    Sampler (uint32_t width, uint32_t height, uint32_t p_width_x, uint32_t p_width_y,
	     uint32_t pixel_num_sample) :
	width(width), height(height), p_width_x(p_width_x), p_width_y(p_width_y),
	pixel_num_sample(pixel_num_sample), sampleset(width * height * pixel_num_sample) {
		p_count_x = width / p_width_x;
		p_count_y = height / p_width_y;
		sampleset.add2Dxy();
    }

    virtual ~Sampler() { }

    virtual Sample *getPacketSamples(uint32_t &x, uint32_t &y, Random462 &rng) = 0;

	void allocate() {
		sampleset.allocateSamples();
	}

	uint32_t add1D(uint32_t oneD) {
		return sampleset.add1D(oneD);
    }

    uint32_t add2D(uint32_t twoD) {
		return sampleset.add2D(twoD);
    }

    uint32_t get_sample_size() {
	return sampleset.sample_size;
    }

    uint32_t width, height;
    uint32_t p_width_x, p_width_y;
    uint32_t p_count_x, p_count_y;
    uint32_t pixel_num_sample;

protected:
    SampleSet sampleset;

};

}

#endif
