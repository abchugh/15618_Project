/**
* @file scene.cpp
* @brief Function definitions for scenes.
*
* @author Eric Butler (edbutler)
* @author Kristin Siu (kasiu)
*/

#include "scene/scene.hpp"
#include "math/random462.hpp"
#include "ray.hpp"
#include <algorithm>
#include "scene/model.hpp"
#include <vector>
#include <SDL_timer.h>
#include <omp.h>

using namespace std;
namespace _462 {

    const real_t SLOP = 1e-5;
    time_t ta[MAX_THREADS], tb[MAX_THREADS], ts[MAX_THREADS], tt[MAX_THREADS], tu[MAX_THREADS];

	Packet::Packet(size_t packet_size) {
            rays = new Ray[packet_size];

            e_x = (float*)memalign(16, sizeof(float) * packet_size);
            e_y = (float*)memalign(16, sizeof(float) * packet_size);
            e_z = (float*)memalign(16, sizeof(float) * packet_size);

            d_x = (float*)memalign(16, sizeof(float) * packet_size);
            d_y = (float*)memalign(16, sizeof(float) * packet_size);
            d_z = (float*)memalign(16, sizeof(float) * packet_size);

            size = packet_size;
        }   

        Packet::~Packet() {
            delete[] rays;

            _aligned_free(e_x);
            _aligned_free(e_y);
            _aligned_free(e_z);
            _aligned_free(d_x);
            _aligned_free(d_y);
            _aligned_free(d_z);
        }

    Geometry::Geometry():
        position(Vector3::Zero()),
        orientation(Quaternion::Identity()),
        scale(Vector3::Ones()),
		light_ptr(NULL)
    {

    }

    Geometry::~Geometry() { }

    void Geometry::InitGeometry()
    {
        make_inverse_transformation_matrix(&invMat, position, orientation, scale);
        make_transformation_matrix(&transMat, position, orientation, scale);
        make_normal_matrix(&normMat, transMat);

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
		tree->get_bounding_box(&world_bounding);
    }

    Geometry* const* Scene::get_geometries() const
    {
        return geometries.empty() ? NULL : &geometries[0];
    }

    size_t Scene::num_geometries() const
    {
        return geometries.size();
    }

    Light* const* Scene::get_lights() const
    {
        return lights.empty() ? NULL : &lights[0];
    }

    size_t Scene::num_lights() const
    {
        return lights.size();
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
        lights.clear();
		simple_lights.clear();

        camera = Camera();

        background_color = Color3::Black();
        ambient_light = Color3::Black();
        refractive_index = 1.0;
    }

    void Scene::calculateDiffuseColors(vector<Vector3>& p,vector<hitRecord>& h, Color3* col) const
    {
        int numRays = p.size();
        
        int indices[1024];
        assert(numRays<=1024);
		time_t startTime;
        for(unsigned int l=0;l<simple_lights.size();l++)
        {
            int numShadowRays = 0;
            vector<Vector3> locs;
            locs.reserve(numRays);
            vector<real_t> NDotLs;
            NDotLs.reserve(numRays);

            startTime = SDL_GetTicks();
            for(int i=0;i<numRays;i++) if(h[i].t>=0)//check if this actually hit something
            {
				Vector3 loc;
				real_t NDotL;
				real_t x = 2 * random_uniform() - 1, y = 2 * random_uniform() - 1, z = 2 * random_uniform() - 1;
				loc = simple_lights[l].position + normalize(Vector3(x,y,z)) * simple_lights[l].radius;
				Vector3 L = normalize(loc-p[i]);
				NDotL = dot(h[i].n,L);

                if(NDotL<=0) continue;
                locs.push_back(loc);
                NDotLs.push_back(NDotL);
                indices[numShadowRays++] = i;
                
            }
			
            Packet pkt(numShadowRays);
            for(int i=0;i<numShadowRays;i++)
            {
                pkt.rays[i].e = p[ indices[i] ];
				float dis = length(locs[i] - simple_lights[0].position);
                pkt.rays[i].d = locs[i] - p[ indices[i] ];
                pkt.e_x[i] = pkt.rays[i].e[0];
                pkt.e_y[i] = pkt.rays[i].e[1];
                pkt.e_z[i] = pkt.rays[i].e[2];
                pkt.d_x[i] = pkt.rays[i].d[0];
                pkt.d_y[i] = pkt.rays[i].d[1];
                pkt.d_z[i] = pkt.rays[i].d[2];
            }
            tt[omp_get_thread_num()] += SDL_GetTicks()-startTime;

            vector<hitRecord> hShadow(numShadowRays);

            startTime = SDL_GetTicks();
            tree->hit(pkt, SLOP, 1 - SLOP, hShadow, false);
            tb[omp_get_thread_num()] += SDL_GetTicks()-startTime;
            
			
            startTime = SDL_GetTicks();
			float dis;
			float dis2;
            for(int i=0;i<numShadowRays;i++)
            {
                //We just want to check if something is between the point and the source
                if(hShadow[i].t<0)
                {
                    Color3 c = simple_lights[l].color;
                    real_t d = sqrt( dot(p[ indices[i] ]-simple_lights[l].position,p[ indices[i] ]-simple_lights[l].position) );
                    c /= simple_lights[l].attenuation.constant + d*simple_lights[l].attenuation.linear + d*d*simple_lights[l].attenuation.quadratic;
                    col[ indices[i] ] += h[indices[i]].mp.diffuse*c*NDotLs[ i ];
                }
				/*
				else {
					dis = length(hShadow[i].t * pkt.rays[i].d + pkt.rays[i].e - simple_lights[0].position);
					dis2 = length(pkt.rays[i].d + pkt.rays[i].e - simple_lights[0].position);
					Vector3 v1 = hShadow[i].t * pkt.rays[i].d + pkt.rays[i].e, v2 = pkt.rays[i].d + pkt.rays[i].e;
					printf("%f <-> %f\n", dis, dis2);
				}
				*/
            }
            tt[omp_get_thread_num()] += SDL_GetTicks()-startTime;
            //average over the simulations
        }
    }

	bool Scene::hit(const Ray& r, const real_t t0, const real_t t1, hitRecord& h, bool fullRecord) const {
		return tree->hit(r, t0, t1, h, fullRecord) != NULL;
	}

	void Scene::hit(const Packet& packet, const real_t t0, const real_t t1, std::vector<hitRecord>& records, bool fullRecord) const {
		return tree->hit(packet, t0, t1, records, fullRecord);
	}

    Color3 Scene::calculateDiffuseColor(Vector3 p,Vector3 n,Color3 kd) const
    {
        /*Number of shadow rays fired to light source*/
        // TODO: more shadow rays to use packet?
        const int sim = 1;
        Color3 col(0,0,0);
        for(unsigned int l=0;l<simple_lights.size();l++)
        {
            Color3 temp(0,0,0);
            for(int s = 0; s<sim; s++)
            {
                real_t x = random_gaussian(), y = random_gaussian(), z = random_gaussian();
                Vector3 loc = simple_lights[l].position + normalize(Vector3(x,y,z)) * simple_lights[l].radius;

                Vector3 L = normalize(loc-p);
                real_t NDotL = dot(n,L);

                if(NDotL>0)
                {
                    Ray shadowRay;
                    shadowRay.e = p;
                    shadowRay.d = loc-p;
                    hitRecord h;

                    //We just want to check if something is between the point and the source
                    if(!tree->hit(shadowRay, SLOP, 1 - 1e-4, h, false))
                    {
                        Color3 c = simple_lights[l].color;
                        real_t d = sqrt( dot(p-simple_lights[l].position,p-simple_lights[l].position) );
                        c /= simple_lights[l].attenuation.constant + d*simple_lights[l].attenuation.linear + d*d*simple_lights[l].attenuation.quadratic;
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

    void Scene::getColors(const Packet& packet, std::vector<std::vector<real_t> >& refractiveStack, Color3* col, int depth, real_t t0, real_t t1) const 
    {
        vector<hitRecord> h(packet.size);

        time_t startTime = SDL_GetTicks();
        tree->hit(packet, t0, t1, h, true);
        ta[omp_get_thread_num()] += SDL_GetTicks()-startTime;
        //Add the ambient component
        
        startTime = SDL_GetTicks();
        vector<Vector3> p(packet.size);
        for(int i=0;i<packet.size;i++)
        {
            if(h[i].t<0)
                col[i] = Scene::background_color;
            else
            {
                if(h[i].mp.refractive_index == 0)
                    col[i] = Scene::ambient_light*h[i].mp.ambient;
				else				
	                col[i] = Color3(0,0,0);
                p[i] = packet.rays[i].e + h[i].t*packet.rays[i].d;
            }
        }

        tu[omp_get_thread_num()] += SDL_GetTicks()-startTime;

        startTime = SDL_GetTicks();
        calculateDiffuseColors(p,h,col);
        ts[omp_get_thread_num()] += SDL_GetTicks()-startTime;
        
        for(int i=0; i<packet.size;i++) if(h[i].t>=0)
        {
            if(depth>0)
            {
                real_t reflectedRatio;
                Color3 reflectedColor(0,0,0);
                Color3 refractedColor(0,0,0);
                if(h[i].mp.specular!=Color3(0,0,0))
                {
                    Ray reflectedRay(p[i],normalize(packet.rays[i].d - 2 *dot(packet.rays[i].d,h[i].n) *h[i].n));
                

                    if(num_glossy_reflection_samples>0)
                    {
                        for(int i=0;i< num_glossy_reflection_samples;i++)
                        {
                            Ray newRay = distortRay(reflectedRay,0.125);
                            reflectedColor += h[i].mp.specular*getColor(newRay, refractiveStack[i], depth-1, SLOP);
                        }
                        reflectedColor /= num_glossy_reflection_samples;
                    }
                    else
                        reflectedColor = h[i].mp.specular*getColor(reflectedRay,refractiveStack[i], depth-1, SLOP);
                }

                //Refraction not possible
                if(h[i].mp.refractive_index == 0)
                    reflectedRatio = 1.0;
                else
                {
                    real_t currentRI = refractiveStack[i].back();
                    if(currentRI == h[i].mp.refractive_index)
                    {
                        if(refractiveStack[i].size()>0)
                        {
                            refractiveStack.pop_back();
                            h[i].mp.refractive_index = refractiveStack[i].back();
                        }
                        else
                            h[i].mp.refractive_index = Scene::refractive_index;
                    }
                    else
                        refractiveStack[i].push_back(h[i].mp.refractive_index);

                    real_t RIRatio = currentRI/h[i].mp.refractive_index;

                    if(RIRatio>1)	//If entering a rarer medium, reverse normal direction
                        h[i].n*=-1;

                    real_t dDotN = dot(normalize(packet.rays[i].d),h[i].n);
                    real_t cosSq = 1-RIRatio*RIRatio*(1-dDotN*dDotN);

                    if(cosSq<0)	//Total Internal Reflection
                    {
                        reflectedRatio = 1.0;
                        refractiveStack[i].push_back(currentRI);
                    }
                    else
                    {
                        real_t cosTheta = sqrt(cosSq);
                        Vector3 dir = (normalize(packet.rays[i].d) - h[i].n *dDotN)*RIRatio - h[i].n*cosTheta; 
                        Ray refractedRay(p[i],dir);

                        refractedColor = getColor(refractedRay, refractiveStack[i], depth-1, SLOP);

                        if(h[i].mp.refractive_index<currentRI)
                            cosTheta = dDotN;
                        real_t normalReflectance = pow( (h[i].mp.refractive_index-1)/(h[i].mp.refractive_index+1),2);
                        reflectedRatio = normalReflectance + normalReflectance *pow(1-cosTheta,5);
                    }
                }
                //Add the two components to the pixel color
                col[i] += reflectedRatio*reflectedColor + (1-reflectedRatio)*refractedColor;
            }
            //Finally multiply by the texture color
            col[i] *= h[i].mp.texColor;
        }
    }

    Color3 Scene::getColor(const Ray& r, std::vector<real_t> refractiveStack, int depth, real_t t0, real_t t1) const
    {
        hitRecord h;
        // Invalid value.
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

                    refractedColor = getColor(refractedRay, refractiveStack, depth-1, SLOP);

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

    void Scene::add_light( Light* l )
    {
        lights.push_back( l );
    }

	const SphereLight* Scene::get_simple_lights() const {
		return simple_lights.empty() ? NULL : &simple_lights[0];
	}

    size_t Scene::num_simple_lights() const {
		return simple_lights.size();
	}

	void Scene::add_simple_light( const SphereLight& l ) {
		simple_lights.push_back( l );
	}

	void Scene::bounding_sphere(Vector3 *center_ptr, float *r_ptr) const {
		*center_ptr = (world_bounding.lowCoord + world_bounding.highCoord) / 2;
		*r_ptr = length(*center_ptr - world_bounding.lowCoord);
	}

} /* _462 */

