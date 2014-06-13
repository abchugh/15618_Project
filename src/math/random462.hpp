
#ifndef _462_RANDOM462_HPP_
#define _462_RANDOM462_HPP_

#include "math/math.hpp"
#include <random>

namespace _462{

class Random462 {
public:
	Random462(uint32_t seed = 5489UL) {
		mti = N+1;
		this->seed(seed);
	}

	void seed(uint32_t seed) const;
    float random() const;
	uint32_t random_int() const;

private:
    static const int N = 624;
    mutable unsigned long mt[N]; /* the array for the state vector  */
    mutable int mti;
};
    /**
     * Generate a uniform random real_t on the interval [0, 1)
     */
    inline real_t random_uniform()
    {
        return real_t(rand())/RAND_MAX;
    }

    /**
     * Generate a uniform random real_t from N(0, 1)
     */
    inline real_t random_gaussian()
    {
        static std::default_random_engine generator;
        static std::normal_distribution<real_t> dist =
            std::normal_distribution<real_t>();
        return dist(generator);
    }


}; // _462

#endif
