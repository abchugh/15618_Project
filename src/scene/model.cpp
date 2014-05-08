/**
 * @file model.cpp
 * @brief Model class
 *
 * @author Eric Butler (edbutler)
 * @author Zeyang Li (zeyangl)
 */

#include "scene/model.hpp"
#include "scene/material.hpp"
#include "application/opengl.hpp"
#include "scene/triangle.hpp"
#include <iostream>
#include <cstring>
#include <string>
#include <fstream>
#include <sstream>


namespace _462 {

Model::Model() : mesh( 0 ), material( 0 ), bvh(NULL) { }
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
void Model::hitPacket(const Packet& packet, int start, int end, real_t t0, real_t *t1, hitRecord* hs, bool fullRecord) const {

}

bool Model::hit(const Ray& r, const real_t t0, const real_t t1,hitRecord& h, bool fullRecord) const
{
	if(!checkBoundingBoxHit(r,t0,t1))
		return false;
	return bvh->hit(r,t0,t1,h,fullRecord) != NULL;
}


} /* _462 */
