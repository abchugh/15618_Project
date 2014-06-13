
#include "sampler.hpp"

namespace _462 {
void latin_hypercube(float *samples, uint32_t dim, uint32_t num, Random462 &rng) {
	float delta = 1.f / num;
	for (uint32_t i = 0; i < num; i++) {
		for (uint32_t j = 0; j < dim; j++) {
			samples[i * dim + j] = (i + rng.random()) * delta;
		}
	}

	for (uint32_t i = 0; i < dim; i++) {
		for (uint32_t j = 0; j < num; j++) {
			uint32_t other = j + (rng.random_int() % (num - j));
			float temp = samples[j * dim + i];
			samples[j * dim + i] = samples[other * dim + i];
			samples[other * dim + i] = temp;
		}
	}
}

Distribution1D::Distribution1D(float *func, uint32_t size)
	: cdf(NULL) {
	cdf = new float[size + 1];
	piece_size = size;

	cdf[0] = 0.f;
	for (uint32_t i = 1; i <= size; i++) {
		cdf[i] = cdf[i - 1] + func[i - 1] / size;
	}

	norm = cdf[size];
	for (uint32_t i = 0; i < size + 1; i++) {
		cdf[i] /= norm;
	}
}

float Distribution1D::sample_continuous(float r, float *pdf) {
	float* interval = std::lower_bound(cdf, cdf + piece_size + 1, r);
	uint32_t offset = std::max(0, interval - cdf - 1);
	float delta = (r - cdf[offset]) / (cdf[offset + 1] - cdf[offset]);
	float func = (cdf[offset + 1] - cdf[offset]) * piece_size;

	// cdf[piece_size] = sum (func)
	if(pdf)
		*pdf = func / norm;

	return (offset + delta) / piece_size;
}

uint32_t Distribution1D::sample_discrete(float r, float *pdf) {
	float* interval = std::lower_bound(cdf, cdf + piece_size + 1, r);
	uint32_t offset = std::max(0, interval - cdf - 1);
	float func = (cdf[offset + 1] - cdf[offset]) * piece_size;

	// cdf[piece_size] = sum (func)
	if(pdf)
		*pdf = func / norm;

	return offset;
}

/////////////////////////////////////

Distribution2D::Distribution2D(float *func, uint32_t size_u, uint32_t size_v) 
	: marginal(NULL) {
	float *marginal_buffer = new float[size_v];

	this->size_u = size_u; this->size_v = size_v;

	for (uint32_t v = 0; v < size_v; v++) {
		conditionals.push_back(new Distribution1D(&func[v * size_u], size_u));
		marginal_buffer[v] = conditionals[v]->norm;
	}

	marginal = new Distribution1D(marginal_buffer, size_v);
	delete[] marginal_buffer;
}

Vector2 Distribution2D::sample_continuous(float r1, float r2, float *pdf) {
	Vector2 result;
	float pdf_u, pdf_v;
	result.y = marginal->sample_continuous(r2, &pdf_v);
	uint32_t v_offset = clamp((uint32_t)(result.y * size_v), 0u, size_v - 1);
	result.x = conditionals[v_offset]->sample_continuous(r1, &pdf_u);

	*pdf = pdf_u * pdf_v;
	return result;
}

float Distribution2D::pdf(float u, float v) {
	uint32_t u_offset = clamp((uint32_t)(u * size_u), 0u, size_u - 1);
	uint32_t v_offset = clamp((uint32_t)(v * size_v), 0u, size_v - 1);
	float func_u = conditionals[v_offset]->cdf[u_offset + 1] - conditionals[v_offset]->cdf[u_offset];
	func_u *= conditionals[v_offset]->piece_size;
	float func_v = marginal->cdf[v_offset + 1] - marginal->cdf[v_offset];
	func_v *= marginal->piece_size;

	if (conditionals[v_offset]->norm * marginal->norm < 1e-3) return 0.f;
	return func_v * func_v / (conditionals[v_offset]->norm * marginal->norm);
}

}

