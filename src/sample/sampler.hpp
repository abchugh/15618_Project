
#ifndef _462_SAMPLER_HPP_
#define _462_SAMPLER_HPP_

#include <vector>
#include <malloc.h>
#include <omp.h>

#include "math/math.hpp"
#include "scene/bvh.hpp"

namespace _462 {

struct Sample {
    float x, y;
};

class SampleSet {
public:
    
    SampleSet() : samples(NULL), sample_capacity(0), current(0) { }
    ~SampleSet() {
	current = 0;
	_aligned_free(samples);
    }

    void add1D(uint32_t oneD) {
	oneD_num.push_back(oneD);
    }

    void add2D(uint32_t twoD) {
	twoD_num.push_back(twoD);
    }

    void allocateSamples() {
	uint32_t count = 0;

	for (int i = 0; i < oneD_num.size(); i++) {
	    count += oneD_num[i];
	}

	for (int i = 0; i < twoD_num.size(); i++) {
	    count += twoD_num[i] * 2;
	}

	sample_size = oneD_num.size() + twoD_num.size() * 2;

	samples = (float*)memalign(16, sizeof(float) * count);
	sample_capacity = count;
    }

    Sample *addSample(float *oneD, float *twoD) {
	uint32_t des = 0;

	des = __sync_fetch_and_add(&current, 1);

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

	des = __sync_fetch_and_add(&current, count);

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

	des = __sync_fetch_and_add(&current, count);

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
};

class Sampler {

public:
    Sampler (uint32_t width, uint32_t height, uint32_t p_width_x, uint32_t p_width_y,
	     uint32_t pixel_num_sample) :
	width(width), height(height), p_width_x(p_width_x), p_width_y(p_width_y),
	pixel_num_sample(pixel_num_sample) {
	p_count_x = width / p_width_x;
	p_count_y = height / p_width_y;
	sampleset.add2D(width * height * pixel_num_sample);
	sampleset.allocateSamples();
    }

    virtual ~Sampler() { }

    virtual Sample *getPacketSamples(uint32_t &x, uint32_t &y) = 0;

    uint32_t width, height;
    uint32_t p_width_x, p_width_y;
    uint32_t p_count_x, p_count_y;
    uint32_t pixel_num_sample;

protected:
    SampleSet sampleset;

};

}

#endif
