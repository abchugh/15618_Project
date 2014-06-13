/**
 * @file [0]
 * @brief Material class
 *
 * @author Eric Butler (edbutler)
 */

#ifndef _462_MATERIAL_MATERIAL_HPP_
#define _462_MATERIAL_MATERIAL_HPP_

#include "math/color.hpp"
#include "math/vector.hpp"
#include "application/opengl.hpp"
#include "bxdf.hpp"
#include <string>

namespace _462 {
class BSDF;

class Material
{
public:

    Material();
    ~Material();

    // ambient color (ignored if refractive_index != 0)
    Color3 ambient;

    // diffuse color
    Color3 diffuse;

    // specular (reflective) color
    Color3 specular;

    // IGNORE THIS FOR THE RAYTRACER, only used for the OpenGL renderer
    real_t shininess;

    // refractive index of material dielectric. 0 is special case for
    // infinity, i.e. opaque. Any other value means transparent with the
    // given refractive index.
    real_t refractive_index;

    // filename of the texture
    std::string texture_filename;

    /**
     * Loads the texture from a file and optionally create a gl texture handle
     * for it. DO NOT CALL EVERY FRAME, as it will re-load the texture each time.
     * @return true on success, false on error.
     */
    bool load();

    /// returns the raw texture data
    const unsigned char* get_texture_data() const;

    /// puts the dimensions into width and height
    void get_texture_size( int* width, int* height ) const;

    /**
     * returns the color of the (x,y) pixel, where
     * x ranges from [0, width-1] and y ranges from [0, height-1].
     * Returns white if there is no texture.
     */
    Color3 get_texture_pixel( int x, int y ) const;

    Color3 get_texture_pixel( real_t x, real_t y ) const;

	Color3 get_texture_pixel_norm( real_t x, real_t y ) const;

	Color3 get_texture_pixel_norm_inv( real_t x, real_t y ) const;

	Color3 get_texture_pixel_norm_inv_gamma( real_t x, real_t y ) const;

	Color3 get_filtered_texture_pixel(int x, int y) const;

    /// Creates opengl data for rendering
    bool create_gl_data();

    /// sets all the gl state for this material
    void set_gl_state() const;

    /// clears out setting that depend on this material, such as the texture.
    /// leaves other settings unchanged for efficiency.
    void reset_gl_state() const;

	_462::BSDF bsdf;

private:

    // dimensions of the texture
    int tex_width, tex_height;

    // raw texture data
    unsigned char* tex_data;

    // opengl descriptor of the texture
    GLuint tex_handle;

    // prevent copy/assignment
    Material( const Material& );
    Material& operator=( const Material& );
};

} /* _462 */

#endif /* _462_MATERIAL_MATERIAL_HPP_ */

