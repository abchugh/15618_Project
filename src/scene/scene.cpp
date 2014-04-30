/**
 * @file scene.cpp
 * @brief Function definitions for scenes.
 *
 * @author Eric Butler (edbutler)
 * @author Kristin Siu (kasiu)
 */

#include "scene/scene.hpp"
#include "math/random462.hpp"
#include <algorithm>
#include "scene/model.hpp"

namespace _462 {

const real_t SLOP = 1e-5;

Geometry::Geometry():
    position(Vector3::Zero()),
    orientation(Quaternion::Identity()),
    scale(Vector3::Ones())
{

}

Geometry::~Geometry() { }

void Geometry::InitGeometry()
{
	make_inverse_transformation_matrix(&invMat, position, orientation, scale);
	Matrix4 mat;
	make_transformation_matrix(&mat, position, orientation, scale);
	make_normal_matrix(&normMat, mat);

	return;
}

bool Geometry::checkBoundingBoxHit(const Ray& r, real_t t0, real_t t1)const
{
	return bb.hit(r,t0,t1);
}

SphereLight::SphereLight():
    position(Vector3::Zero()),
    color(Color3::White()),
	radius(real_t(0))
{
    attenuation.constant = 1;
    attenuation.linear = 0;
    attenuation.quadratic = 0;
}

const int Scene::maxRecursionDepth = 3;

Scene::Scene()
{
	tree = NULL;
    reset();
}

Scene::~Scene()
{
    reset();
}

void Scene::InitGeometry()
{
	for (unsigned int i = 0; i < num_geometries(); i++)
		geometries[i]->InitGeometry();
}

void Geometry::Transform(real_t translate, const Vector3 rotate)
{
	if(translate!=0)
	{
		Vector3 translation(translate,0,0);
		if(orientation!=Quaternion::Zero())
			translation = orientation*translation;
		position += translation;
	}
	else
	{
		Quaternion newQuart(rotate,PI/180);		//10 degree rotations
		orientation = orientation*newQuart;
	}
	
	InitGeometry();
}

void Scene::buildBVH()
{
	tree = new BVHAccel(geometries);
}

Geometry* const* Scene::get_geometries() const
{
    return geometries.empty() ? NULL : &geometries[0];
}

size_t Scene::num_geometries() const
{
    return geometries.size();
}

const SphereLight* Scene::get_lights() const
{
    return point_lights.empty() ? NULL : &point_lights[0];
}

size_t Scene::num_lights() const
{
    return point_lights.size();
}

Material* const* Scene::get_materials() const
{
    return materials.empty() ? NULL : &materials[0];
}

size_t Scene::num_materials() const
{
    return materials.size();
}

Mesh* const* Scene::get_meshes() const
{
    return meshes.empty() ? NULL : &meshes[0];
}

size_t Scene::num_meshes() const
{
    return meshes.size();
}

void Scene::reset()
{
    for ( GeometryList::iterator i = geometries.begin(); i != geometries.end(); ++i ) {
        delete *i;
    }
    for ( MaterialList::iterator i = materials.begin(); i != materials.end(); ++i ) {
        delete *i;
    }
    for ( MeshList::iterator i = meshes.begin(); i != meshes.end(); ++i ) {
        delete *i;
    }

    if(tree) {
	delete tree;
	tree = NULL;
    }
    geometries.clear();
    materials.clear();
    meshes.clear();
    point_lights.clear();

    camera = Camera();

    background_color = Color3::Black();
    ambient_light = Color3::Black();
    refractive_index = 1.0;
}

Color3 Scene::calculateDiffuseColor(Vector3 p,Vector3 n,Color3 kd) const
{
	/*Number of shadow rays fired to light source*/
	const int sim = 1;
	Color3 col(0,0,0);
	for(unsigned int l=0;l<point_lights.size();l++)
	{
		Color3 temp(0,0,0);
		for(int s = 0; s<sim; s++)
		{
			real_t x = random_gaussian(), y = random_gaussian(), z = random_gaussian();
			Vector3 loc = point_lights[l].position + normalize(Vector3(x,y,z)) * point_lights[l].radius;
			
			Vector3 L = normalize(loc-p);
			real_t NDotL = dot(n,L);

			if(NDotL>0)
			{
				Ray shadowRay;
				shadowRay.e = p;
				shadowRay.d = loc-p;
				hitRecord h;
				
				//We just want to check if something is between the point and the source
				if(!tree->hit(shadowRay, SLOP, 1, h, false))
				{
					Color3 c = point_lights[l].color;
					real_t d = sqrt( dot(p-point_lights[l].position,p-point_lights[l].position) );
					c /= point_lights[l].attenuation.constant + d*point_lights[l].attenuation.linear + d*d*point_lights[l].attenuation.quadratic;
					temp += kd*c*NDotL;
				}
			}
		}
		//average over the simulations
		col += temp/sim;
	}
	return col;
}
static Ray distortRay(Ray r, real_t distortionWidth)
{
	//return r;
	real_t u = random_gaussian()*distortionWidth, v = random_gaussian()*distortionWidth;
	u -= distortionWidth/2;
	v -= distortionWidth/2;
	
	Vector3 dir1 = cross(r.d +Vector3(0,0,1),r.d);
	Vector3 dir2 = cross(dir1,r.d);

	r.d = normalize(r.d + u*dir1 + v*dir2);
	return r;
}

void Scene::handleClick(int x, int y, int width, int height, int translation)
{
	y = height - y;
	real_t dx = real_t(1)/width;
	real_t dy = real_t(1)/height;
	
    real_t i = real_t(2)*(real_t(x))*dx - real_t(1);
    real_t j = real_t(2)*(real_t(y))*dy - real_t(1);

	Ray::init(camera);
    Ray r = Ray(camera.get_position(), Ray::get_pixel_dir(i, j));
		
	hitRecord h;
	Geometry* obj;
	if((obj = tree->hit(r, 0, BIG_NUMBER, h, true)))
	{
		obj->Transform(translation,Vector3(0,0,0));
		if(tree)
		{
			delete tree;
			buildBVH();
		}
	}
}
void Scene::TransformModels(real_t translate, const Vector3 rotate)
{
	for(unsigned int i=0;i<geometries.size();i++)
	{
		Model* model = dynamic_cast<Model*>(geometries[i]);
		if(model)
			model->Transform(translate,rotate);
	}
	if(tree)
	{
		delete tree;
		buildBVH();
	}
}
Color3 Scene::getColor(const Ray& r, std::vector<real_t> refractiveStack, int depth, real_t t0, real_t t1) const
{
	hitRecord h;
	
	if(!tree->hit(r, t0, t1, h, true))
		return Scene::background_color;	//Nothing hit, return background color

	//Add the ambient component
	Color3 col(0,0,0);
	if(h.mp.refractive_index == 0)
		col += Scene::ambient_light*h.mp.ambient;

	//Add the diffuse component
	Vector3 p = r.e + h.t*r.d;
	if(h.mp.refractive_index == 0)
		col += calculateDiffuseColor(p,h.n,h.mp.diffuse);

	if(depth>0)
	{
		real_t reflectedRatio;
		Color3 reflectedColor(0,0,0);
		Color3 refractedColor(0,0,0);
		if(h.mp.specular!=Color3(0,0,0))
		{
			Ray reflectedRay(p,normalize(r.d - 2 *dot(r.d,h.n) *h.n));
				
			if(num_glossy_reflection_samples>0)
			{
				for(int i=0;i< num_glossy_reflection_samples;i++)
				{
					Ray newRay = distortRay(reflectedRay,0.125);
					reflectedColor += h.mp.specular*getColor(newRay, refractiveStack, depth-1, SLOP);
				}
				reflectedColor /= num_glossy_reflection_samples;
			}
			else
				reflectedColor = h.mp.specular*getColor(reflectedRay,refractiveStack, depth-1, SLOP);
		}
		
		//Refraction not possible
		if(h.mp.refractive_index == 0)
			reflectedRatio = 1.0;
		else
		{
			real_t currentRI = refractiveStack.back();
			if(currentRI == h.mp.refractive_index)
			{
				if(refractiveStack.size()>0)
				{
					refractiveStack.pop_back();
					h.mp.refractive_index = refractiveStack.back();
				}
				else
					h.mp.refractive_index = Scene::refractive_index;
			}
			else
				refractiveStack.push_back(h.mp.refractive_index);
			
			real_t RIRatio = currentRI/h.mp.refractive_index;
			
			if(RIRatio>1)	//If entering a rarer medium, reverse normal direction
				h.n*=-1;
			
			real_t dDotN = dot(normalize(r.d),h.n);
			real_t cosSq = 1-RIRatio*RIRatio*(1-dDotN*dDotN);

			if(cosSq<0)	//Total Internal Reflection
			{
				reflectedRatio = 1.0;
				refractiveStack.push_back(currentRI);
			}
			else
			{
				real_t cosTheta = sqrt(cosSq);
				Vector3 dir = (normalize(r.d) - h.n *dDotN)*RIRatio - h.n*cosTheta; 
				Ray refractedRay(p,dir);
				
				refractedColor = getColor(refractedRay, refractiveStack,depth-1, SLOP);

				if(h.mp.refractive_index<currentRI)
					cosTheta = dDotN;
				real_t normalReflectance = pow( (h.mp.refractive_index-1)/(h.mp.refractive_index+1),2);
				reflectedRatio = normalReflectance + normalReflectance *pow(1-cosTheta,5);
			}
		}
		//Add the two components to the pixel color
		col += reflectedRatio*reflectedColor + (1-reflectedRatio)*refractedColor;
	}
	//Finally multiply by the texture color
	return col*h.mp.texColor;
}

void Scene::add_geometry( Geometry* g )
{
    geometries.push_back( g );
}

void Scene::add_material( Material* m )
{
    materials.push_back( m );
}

void Scene::add_mesh( Mesh* m )
{
    meshes.push_back( m );
}

void Scene::add_light( const SphereLight& l )
{
    point_lights.push_back( l );
}


} /* _462 */

