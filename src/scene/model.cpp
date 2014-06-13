/**
 * @file model.cpp
 * @brief Model class
 *
 * @author Eric Butler (edbutler)
 * @author Zeyang Li (zeyangl)
 */

#include "scene/model.hpp"
#include "material/material.hpp"
#include "application/opengl.hpp"
#include "scene/triangle.hpp"
#include <iostream>
#include <cstring>
#include <string>
#include <fstream>
#include <sstream>


namespace _462 {

Model::Model() : mesh( 0 ), material( 0 ), bvh(NULL), sum_area(-1) { }
Model::~Model()
{ 
	if(bvh)
	{
		delete bvh; 
		bvh=NULL;
	} 
}

void Model::render() const
{
    if ( !mesh )
        return;
    if ( material )
        material->set_gl_state();
    mesh->render();
    if ( material )
        material->reset_gl_state();
}
void Model::InitGeometry()
{ 
	if(bvh)
	{
		delete bvh;
		bvh = NULL;
	}
	
	if (mesh == NULL)
		return ;

	triangles.clear();

	Geometry::InitGeometry();
	Matrix4 mat;
	make_transformation_matrix(&mat, position,orientation,scale);
	
	for(unsigned int i=0;i<mesh->num_vertices();i++)
		bb.AddPoint(project(mat*Vector4(mesh->vertices[i].position,1)));

	std::vector<Geometry*> geometries; 
	triangles.reserve(mesh->num_triangles());
	
	const MeshTriangle* mTriangles = mesh->get_triangles();
	const MeshVertex* mVertices = mesh->get_vertices();

	for(unsigned int i=0;i<mesh->num_triangles();i++)
	{
		Triangle t;
		t.invMat = invMat;
		t.normMat = normMat;
		t.orientation = orientation;
		t.position = position;
		t.scale = scale;
		t.simple = true;
		const unsigned int vertexIndices[] = {mTriangles[i].vertices[0], mTriangles[i].vertices[1], mTriangles[i].vertices[2] };
		MeshVertex tVertex[] = {mVertices[ vertexIndices[0] ], mVertices[ vertexIndices[1] ], mVertices[ vertexIndices[2] ] };
		
		for(int j=0;j<3;j++)
		{
			t.vertices[j].material = material;
			t.vertices[j].position = tVertex[j].position;
			t.vertices[j].normal = tVertex[j].normal;
			t.vertices[j].tex_coord = tVertex[j].tex_coord;
		}
		t.InitGeometry();
		triangles.push_back(t);
		geometries.push_back(&triangles[i]);
	}
	bvh = new BVHAccel(geometries);
}

    // TODO: model's hitpacket
void Model::hitPacket(const Packet& packet, int start, int end, real_t t0, real_t *t1, std::vector<hitRecord>& hs, bool fullRecord) const {
  
    for(int i=start; i<end; i++)
        bvh->hit(packet.rays[i], t0, t1[i], hs[i], fullRecord);
}

bool Model::hit(const Ray& r, const real_t t0, const real_t t1,hitRecord& h, bool fullRecord) const
{
	if(!checkBoundingBoxHit(r,t0,t1))
		return false;
	bool hit = (bvh->hit(r,t0,t1,h,fullRecord) != NULL);
	if (hit)
		h.shape_ptr = const_cast<Model*>(this);
	if (hit && fullRecord) {
		h.bsdf_ptr = const_cast<BSDF*>(&(material->bsdf));
		
		Vector3 x, y, z = h.n;
		coordinate_system(z, &x, &y);
		h.shading_trans = Matrix3(x, y, z);
	}
	
	return hit;
}

float Model::get_area() {
	if (sum_area > 0)
		return sum_area;
	float area = 0;

	for (unsigned int i = 0; i < triangles.size(); i++) {
		area += triangles[i].get_area();
	}

	sum_area = area;
	return area;
}
	
Vector3 Model::sample(const Vector3 &p, float r1, float r2, float c, Vector3 *n_ptr) {
	int id = (int) triangles.size() * c;

	if (id >= triangles.size())
		id = triangles.size() - 1;

	return triangles[id].sample(p, r1, r2, c, n_ptr);
}

float Model::pdf(const Vector3 &p, const Vector3 &wi) {
	float pdf = 0.f;
	for (uint32_t i = 0; i < triangles.size(); i++) {
		pdf += triangles[i].get_area() * triangles[i].pdf(p, wi);
	}

	return pdf / get_area();
}

} /* _462 */
