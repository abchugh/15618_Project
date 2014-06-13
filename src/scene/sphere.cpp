/**
 * @file sphere.cpp
 * @brief Function defnitions for the Sphere class.
 *
 * @author Kristin Siu (kasiu)
 * @author Eric Butler (edbutler)
 */

#include "scene/sphere.hpp"
#include "scene/ray.hpp"
#include "sample/sampler.hpp"
#include "application/opengl.hpp"
#include "material/material.hpp"

namespace _462 {

#define SPHERE_NUM_LAT 80
#define SPHERE_NUM_LON 100

#define SPHERE_NUM_VERTICES ( ( SPHERE_NUM_LAT + 1 ) * ( SPHERE_NUM_LON + 1 ) )
#define SPHERE_NUM_INDICES ( 6 * SPHERE_NUM_LAT * SPHERE_NUM_LON )
// index of the x,y sphere where x is lat and y is lon
#define SINDEX(x,y) ((x) * (SPHERE_NUM_LON + 1) + (y))
#define VERTEX_SIZE 8
#define TCOORD_OFFSET 0
#define NORMAL_OFFSET 2
#define VERTEX_OFFSET 5

static unsigned int Indices[SPHERE_NUM_INDICES];
static float Vertices[VERTEX_SIZE * SPHERE_NUM_VERTICES];

static void init_sphere()
{
    static bool initialized = false;
    if ( initialized )
        return;

    for ( int i = 0; i <= SPHERE_NUM_LAT; i++ ) {
        for ( int j = 0; j <= SPHERE_NUM_LON; j++ ) {
            real_t lat = real_t( i ) / SPHERE_NUM_LAT;
            real_t lon = real_t( j ) / SPHERE_NUM_LON;
            float* vptr = &Vertices[VERTEX_SIZE * SINDEX(i,j)];

            vptr[TCOORD_OFFSET + 0] = lon;
            vptr[TCOORD_OFFSET + 1] = 1-lat;

            lat *= PI;
            lon *= 2 * PI;
            real_t sinlat = sin( lat );

            vptr[NORMAL_OFFSET + 0] = vptr[VERTEX_OFFSET + 0] = sinlat * sin( lon );
            vptr[NORMAL_OFFSET + 1] = vptr[VERTEX_OFFSET + 1] = cos( lat ),
            vptr[NORMAL_OFFSET + 2] = vptr[VERTEX_OFFSET + 2] = sinlat * cos( lon );
        }
    }

    for ( int i = 0; i < SPHERE_NUM_LAT; i++ ) {
        for ( int j = 0; j < SPHERE_NUM_LON; j++ ) {
            unsigned int* iptr = &Indices[6 * ( SPHERE_NUM_LON * i + j )];

            unsigned int i00 = SINDEX(i,  j  );
            unsigned int i10 = SINDEX(i+1,j  );
            unsigned int i11 = SINDEX(i+1,j+1);
            unsigned int i01 = SINDEX(i,  j+1);

            iptr[0] = i00;
            iptr[1] = i10;
            iptr[2] = i11;
            iptr[3] = i11;
            iptr[4] = i01;
            iptr[5] = i00;
        }
    }

    initialized = true;
}

Sphere::Sphere()
    : radius(0), material(0) {}

Sphere::~Sphere() {}

void Sphere::render() const
{
    // create geometry if we haven't already
    init_sphere();

    if ( material )
        material->set_gl_state();

    // just scale by radius and draw unit sphere
    glPushMatrix();
    glScaled( radius, radius, radius );
    glInterleavedArrays( GL_T2F_N3F_V3F, VERTEX_SIZE * sizeof Vertices[0], Vertices );
    glDrawElements( GL_TRIANGLES, SPHERE_NUM_INDICES, GL_UNSIGNED_INT, Indices );
    glPopMatrix();

    if ( material )
        material->reset_gl_state();
}

void Sphere::InitGeometry()
{
	Geometry::InitGeometry();
	Matrix4 mat;
	make_transformation_matrix(&mat, position,orientation,scale);
	real_t r = radius;
	Vector3 endPoints[] = {Vector3(r,r,r), Vector3(r,r,-r),  Vector3(r,-r,r), Vector3(r,-r,-r),Vector3(-r,r,r), Vector3(-r,r,-r),Vector3(-r,-r,r),Vector3(-r,-r,-r)};	

	for(int i=0;i<8;i++)
		bb.AddPoint(project(mat*Vector4(endPoints[i],1)));
}

    // TODO: sphere's hitpacket
void Sphere::hitPacket(const Packet& packet, int start, int end, real_t t0, real_t *t1Ptr, std::vector<hitRecord>& hs, bool fullRecord) const {
	for (uint32_t i = start; i < end; i++) {
		this->hit(packet.rays[i], t0, t1Ptr[i], hs[i], fullRecord);
	}
}

float Sphere::get_area() {
	return 4 * PI * radius * radius;
}

Vector3 Sphere::sample(float r1, float r2, Vector3 *n_ptr) {
	*n_ptr = normalize(uniform_sample_sphere(r1, r2));
	return *n_ptr * radius + position;
}

Vector3 Sphere::sample(const Vector3 &p, float r1, float r2, float c, Vector3 *n_ptr) {
	if (squared_length(position - p) < radius * radius) {
		return sample(r1, r2, n_ptr);
	}
	
	Vector3 sample_n = normalize(position - p);
	Vector3 x, y;
	coordinate_system(sample_n, &x, &y);
	
	float v1 = dot(x, sample_n), v2 = dot(x, y), v3 = dot(y, sample_n);
	float test = squared_length(position - p);

	float cos_max = std::sqrt(1.f - radius * radius / squared_length(position - p));
	Vector3 p_cone = uniform_sample_cone(r1, r2, cos_max, x, y, sample_n);
	Ray r(p, p_cone);

	hitRecord h;
	float time;
	if(!this->hit(r, 1e-3, BIG_NUMBER, h, true))
		time = dot(position - p, normalize(p_cone - p));
	else
		time = h.t;
	p_cone = r.e + time * r.d;
	*n_ptr = normalize(p_cone - position);

	return p_cone;
}

float Sphere::pdf(const Vector3 &p, const Vector3 &wi) {
	if (squared_length(position - p) < radius * radius) {
		return uniform_sample_sphere_pdf();
	}
	float cos_max = std::sqrt(1.f - radius * radius / squared_length(position - p));
	return uniform_sample_cone_pdf(cos_max);
}

bool Sphere::hit(const Ray& r, const real_t t0, const real_t t1, hitRecord & h, bool fullRecord) const
{
	if(!checkBoundingBoxHit(r,t0,t1))
		return false;

	Ray tRay = r.transform(invMat); //transformed ray
	Vector3 d = tRay.d;
	Vector3 e = tRay.e;
	Vector3 c = Vector3::Zero();	
	real_t R = radius;
	
	real_t A = dot(d,d);
	real_t B = 2*dot(d,e-c);
	real_t C = dot(e-c,e-c)  - R*R;
	real_t D = B*B - 4*A*C;
	if(D<0)
	    return false;
	else
	{
        real_t t = (-B -sqrt(D) )/ (2 * A);
	    if(t<t0)
		    t = (-B + sqrt(D) )/ (2 * A);
		
	    if(t<t0 || t>t1)
		    return false;
        h.t = t;
	    if(fullRecord)
		{
			Vector3 p = e + h.t*d;
			h.n = normalize(normMat*(p-c));

			Vector3 x, y, z = h.n;
			coordinate_system(z, &x, &y);

			h.shading_trans = Matrix3(x, y, z);
			inverse(&h.inv_shading_trans, h.shading_trans);

			if (material != NULL) {
				h.mp.diffuse = material->diffuse;
				h.mp.ambient = material->ambient;
				h.mp.specular = material->specular;
				h.mp.refractive_index = material->refractive_index;
			
				h.mp.texColor = Color3(1,1,1);
				if(material->get_texture_data())
				{
					int width,height;
					material->get_texture_size(&width, &height );

					real_t x = p.z;
					real_t y = p.x;
					real_t z = p.y;
					Vector3 v = Vector3(x,y,z);
					project( invMat*Vector4(v,1));

					real_t theta = acos(v.z/radius	);
					real_t phi = atan2(v.y,v.x);
					if(phi<0)
						phi += 2*PI;

					h.mp.texColor = material->get_texture_pixel( (phi/(2*PI))*width, ((PI-theta)/PI)*height);
				}
				h.bsdf_ptr = (BSDF*)&(material->bsdf);
			}
		}
		h.shape_ptr = (Geometry*)this;
		h.p = r.d * t + r.e;

		return true;
	}

}
} /* _462 */

