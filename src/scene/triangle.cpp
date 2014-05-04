/**
 * @file triangle.cpp
 * @brief Function definitions for the Triangle class.
 *
 * @author Eric Butler (edbutler)
 */

#include "scene/triangle.hpp"
#include "application/opengl.hpp"

namespace _462 {

Triangle::Triangle()
{
    vertices[0].material = 0;
    vertices[1].material = 0;
    vertices[2].material = 0;
	
	simple = false;
}

Triangle::~Triangle() { }

void Triangle::render() const
{
    bool materials_nonnull = true;
    for ( int i = 0; i < 3; ++i )
        materials_nonnull = materials_nonnull && vertices[i].material;

    // this doesn't interpolate materials. Ah well.
    if ( materials_nonnull )
        vertices[0].material->set_gl_state();

    glBegin(GL_TRIANGLES);

    glNormal3dv( &vertices[0].normal.x );
    glTexCoord2dv( &vertices[0].tex_coord.x );
    glVertex3dv( &vertices[0].position.x );

    glNormal3dv( &vertices[1].normal.x );
    glTexCoord2dv( &vertices[1].tex_coord.x );
    glVertex3dv( &vertices[1].position.x);

    glNormal3dv( &vertices[2].normal.x );
    glTexCoord2dv( &vertices[2].tex_coord.x );
    glVertex3dv( &vertices[2].position.x);

    glEnd();

    if ( materials_nonnull )
        vertices[0].material->reset_gl_state();
}

bool Triangle::getBarycentricCoordinates(const Ray& r, real_t& t,real_t mult[3], Vector3 position[3])
{
	Matrix3 A;
	Vector3 b,sol;
	for(int i=0;i<3;i++)
	{
		A(0,i) = position[0][i] - position[1][i];
		A(1,i) = position[0][i] - position[2][i]; 
		A(2,i) = r.d[i];
		b[i] =  position[0][i] - r.e[i];
	}
	
	if(!solveLinearSystem(A, b, sol))
		return false;
	t = sol[2];

	mult[0] =	1-sol[0]-sol[1];
	mult[1] =	sol[0];
	mult[2] =	sol[1];
	return (sol[0]+sol[1]) <= 1 && sol[0]>=0 && sol[1]>=0;
}

void Triangle::getMaterialProperties(MaterialProp& mp, const real_t mult[3],const Vector2& texCoord, const Material* materials[3])
{
	mp.diffuse = Color3(0,0,0);
	mp.ambient = Color3(0,0,0);
	mp.specular= Color3(0,0,0);
	mp.refractive_index= 0;
	mp.texColor = Color3(0,0,0);

	
	for ( int i = 0; i < 3; ++i )
	{
		if(materials[i])
		{
			mp.diffuse += materials[i]->diffuse*mult[i];
			mp.ambient += materials[i]->ambient*mult[i];
			mp.specular += materials[i]->specular*mult[i];
			mp.refractive_index += materials[i]->refractive_index*mult[i];
			
			if(materials[i]->get_texture_data())
			{
				int width,height;
				materials[i]->get_texture_size(&width, &height );

				mp.texColor += mult[i]*materials[i]->get_texture_pixel( texCoord[0]*width, texCoord[1]*height);
			}
			else
				mp.texColor+=Color3(1,1,1)*mult[i];
		}
	}
}

void Triangle::getMaterialProperties(MaterialProp& mp, const Vector2& texCoord, const Material* material)
{
	mp.diffuse = material->diffuse;
	mp.ambient = material->ambient;
	mp.specular = material->specular;
	mp.refractive_index = material->refractive_index;
	mp.texColor = Color3(1,1,1);

	if(material->get_texture_data())
	{
		int width,height;
		material->get_texture_size(&width, &height );
		mp.texColor = material->get_texture_pixel( texCoord[0]*width,texCoord[1]*height);
	}
}

void Triangle::InitGeometry()
{
	Geometry::InitGeometry();
	Matrix4 mat;
	make_transformation_matrix(&mat, position,orientation,scale);
	
	for(int i=0;i<3;i++)
		bb.AddPoint(project(mat*Vector4(vertices[i].position,1)));
}

bool Triangle::hit(const Ray& r, const real_t t0, const real_t t1, hitRecord& h, bool fullRecord) const
{
//	if(!checkBoundingBoxHit(r,t0,t1))
	//	return false;

	Ray tRay = r.transform(invMat);
	real_t mult[3];
	Vector3 position[] = { vertices[0].position, vertices[1].position, vertices[2].position };
	const Material* materials[] = { vertices[0].material,vertices[1].material,vertices[2].material };
    real_t t;
	if(!getBarycentricCoordinates(tRay,t,mult,position))
		return false;

	if(t<t0 || t>t1)
		return false;
    h.t = t;
	if(!fullRecord)
		return true;	
	
	Vector2 texCoord(0,0);
	for(int i=0;i<3;i++)
		texCoord += mult[i]*vertices[i].tex_coord;

	texCoord[0] = fmod(texCoord[0],1.0);
	texCoord[1] = fmod(texCoord[1],1.0);
	if(texCoord[0]<0) texCoord[0]+=1;
	if(texCoord[1]<0) texCoord[0]+=1;
	
	if(!simple)
		getMaterialProperties(h.mp, mult,texCoord, materials);
	else
		getMaterialProperties(h.mp,texCoord, materials[0]);

	h.n = Vector3(0,0,0);
	for(int i=0;i<3;i++)
	    h.n += mult[i]*vertices[i].normal;
	h.n = normalize( normMat*h.n);

	
	return true;
}

} /* _462 */
