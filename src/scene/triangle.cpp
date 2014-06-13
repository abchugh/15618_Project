/**
* @file triangle.cpp
* @brief Function definitions for the Triangle class.
*
* @author Eric Butler (edbutler)
*/

#include "scene/triangle.hpp"
#include "application/opengl.hpp"
#include "material/material.hpp"

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
		mp.diffuse = Color3(0,0,0);
        mp.ambient = Color3(0,0,0);
        mp.specular= Color3(0,0,0);
        mp.refractive_index= 0;
        mp.texColor = Color3(0,0,0);

		if (material) {
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
    }

    void Triangle::InitGeometry()
    {
        Geometry::InitGeometry();
        Matrix4 mat;
        make_transformation_matrix(&mat, position,orientation,scale);

        for(int i=0;i<3;i++)
            bb.AddPoint(project(mat*Vector4(vertices[i].position,1)));
    }

    void Triangle::hitPacket(const Packet& packet, int start, int end, real_t t0, real_t *t1Ptr, std::vector<hitRecord>& hs, bool fullRecord) const {
        /*Matrix4 nm = this->invMat;
        nm(3,0)=0;
        nm(3,1)=0;
        nm(3,2)=0;

        Ray rays[256];
        for(int i=start;i<end;i++)
        {
            rays[i] = packet.rays[i];
            packet.rays[i].transform(invMat);
        }*/
        // TODO: static
        float *texCoord_x, *texCoord_y, 
            *norm_x, *norm_y, *norm_z;

        texCoord_x = new float[end - start];
        texCoord_y = new float[end - start];
        norm_x = new float[end - start];
        norm_y = new float[end - start];
        norm_z = new float[end - start];

        int *hit_flag = new int[end - start];

        ispc::hit_triangle(packet.e_x, packet.e_y, packet.e_z, packet.d_x, packet.d_y, packet.d_z,
            t0, t1Ptr, 
            (double*)&vertices[0],(double*)&vertices[1],(double*)&vertices[2], (this->invMat._m),
            start, end, (int)fullRecord, 
            hit_flag, texCoord_x, texCoord_y, norm_x, norm_y, norm_z);
        const Material* materials[] = { vertices[0].material,vertices[1].material,vertices[2].material };

        for (int i = start; i < end; i++) {
            if (hit_flag[i - start]) {
                hs[i].t = t1Ptr[i];
                Vector2 texCoord(texCoord_x[i - start], texCoord_y[i - start]);

                // TODO: interpolation
                //if(!simple)
                //getMaterialProperties(hs[i].mp, mult, texCoord, materials);
                //else
                getMaterialProperties(hs[i].mp,texCoord, materials[0]);
                hs[i].n = Vector3(norm_x[i - start], norm_y[i - start], norm_z[i - start]);
				if (materials[0])
					hs[i].bsdf_ptr = (BSDF*)&(materials[0]->bsdf);
				Vector3 x, y, z = hs[i].n;
				coordinate_system(z, &x, &y);
			
				hs[i].shading_trans = Matrix3(x, y, z);
				inverse(&hs[i].inv_shading_trans, hs[i].shading_trans);
				hs[i].shape_ptr = (Geometry*)this;
				hs[i].p = packet.rays[i].d * hs[i].t + packet.rays[i].e;
            }
        }

        /*for(int i=start;i<end;i++)
             packet.rays[i]= rays[i];*/

        delete[] texCoord_x;
        delete[] texCoord_y;
        delete[] norm_x;
        delete[] norm_y;
        delete[] norm_z;
    }
    
	float Triangle::get_area() {
		Vector3 a = vertices[1].position - vertices[0].position;
        Vector3 b = vertices[2].position - vertices[0].position;

		return length(cross(a, b)) * 0.5f;
	}

	Vector3 Triangle::sample(const Vector3 &p, float r1, float r2, float c, Vector3 *n_ptr) {
		Vector2 tri_sample = uniform_sample_triangle(r1, r2);
		float alpha = tri_sample.x;
		float beta = tri_sample.y;
		float gamma = 1 - alpha - beta;

		Vector3 result = alpha * vertices[0].position + beta * vertices[1].position + gamma * vertices[2].position;
		*n_ptr = alpha * vertices[0].normal + beta * vertices[1].normal + gamma * vertices[2].normal;
		*n_ptr = normalize(*n_ptr);

		return result;
	}

	float Triangle::pdf(const Vector3 &p, const Vector3 &wi) {
		return 1.f / get_area();
	}

    bool Triangle::hit(const Ray& r, const real_t t0, const real_t t1, hitRecord& hR, bool fullRecord) const
    {
        Ray tRay = r.transform(invMat);

        real_t mult[3];

        Vector3 a_minus_b = vertices[0].position - vertices[1].position;
        Vector3 a_minus_c = vertices[0].position - vertices[2].position;
        Vector3 a_minus_e = vertices[0].position - tRay.e;

        real_t a = a_minus_b.x;
        real_t b = a_minus_b.y;
        real_t c = a_minus_b.z;
        real_t d = a_minus_c.x;
        real_t e = a_minus_c.y;
        real_t f = a_minus_c.z;
        real_t g = tRay.d.x;
        real_t h = tRay.d.y;
        real_t i = tRay.d.z;
        real_t j = a_minus_e.x;
        real_t k = a_minus_e.y;
        real_t l = a_minus_e.z;

        real_t ei_minus_hf = e * i - h * f;
        real_t gf_minus_di = g * f - d * i;
        real_t dh_minus_eg = d * h - e * g;
        real_t ak_minus_jb = a * k - j * b;
        real_t jc_minus_al = j * c - a * l;
        real_t bl_minus_kc = b * l - k * c;

        real_t M = a * ei_minus_hf + b * gf_minus_di + c * dh_minus_eg;
        real_t time = (f * ak_minus_jb + e * jc_minus_al + d * bl_minus_kc) / -M;
        if (time <= t0 || time >= t1) {
            return false;
        }

        real_t beta = (j * ei_minus_hf + k * gf_minus_di + l * dh_minus_eg) / M;
        if (beta < 0 || beta > 1)
            return false;

        real_t gamma = (i * ak_minus_jb + h * jc_minus_al + g * bl_minus_kc) / M;
        if (gamma < 0 || gamma > 1 - beta)
            return false;

        hR.t = time;
		hR.shape_ptr = (Geometry*)this;

        if (!fullRecord)
            return true;

        mult[0] = 1-beta-gamma;
        mult[1] = beta;
        mult[2] = gamma;

        Vector2 texCoord(0,0);
        for(int i=0;i<3;i++)
            texCoord += mult[i]*vertices[i].tex_coord;

        texCoord[0] = fmod(texCoord[0],1.0);
        texCoord[1] = fmod(texCoord[1],1.0);
        if(texCoord[0]<0) texCoord[0]+=1;
        if(texCoord[1]<0) texCoord[0]+=1;

        const Material* materials[] = { vertices[0].material,vertices[1].material,vertices[2].material };
        if(!simple)
            getMaterialProperties(hR.mp, mult, texCoord, materials);
        else
            getMaterialProperties(hR.mp,texCoord, materials[0]);

        hR.n = Vector3(0,0,0);
        for(int i=0;i<3;i++)
            hR.n += mult[i]*vertices[i].normal;
        hR.n = normalize( normMat*hR.n);

		if (materials[0])
			hR.bsdf_ptr = (BSDF*)&(materials[0]->bsdf);
		Vector3 x, y, z = hR.n;
		coordinate_system(z, &x, &y);
			
		hR.shading_trans = Matrix3(x, y, z);
		inverse(&hR.inv_shading_trans, hR.shading_trans);
		hR.p = r.d * time + r.e;

        return true;
    }

} /* _462 */
