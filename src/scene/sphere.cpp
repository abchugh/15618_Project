/**
 * @file sphere.cpp
 * @brief Function defnitions for the Sphere class.
 *
 * @author Kristin Siu (kasiu)
 * @author Eric Butler (edbutler)
 */

#include "scene/sphere.hpp"
#include "application/opengl.hpp"

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
	    h.t = (-B -sqrt(D) )/ (2 * A);
	    if(h.t<t0)
		h.t = (-B + sqrt(D) )/ (2 * A);
		
	    if(h.t<t0 || h.t>t1)
		return false;

	    if(fullRecord)
		{
			Vector3 p = e + h.t*d;
			h.n = normalize(normMat*(p-c));
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
		}
		return true;
	}

}
} /* _462 */

