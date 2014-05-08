
typedef float<2> Vector2;
typedef float<3> Vector3;
typedef float<4> Vector4;

#define BIG_NUMBER 1000000
#define TRI_VEC_SIZE 72

Vector3 normalize(Vector3 v) {
    float length = 0.f;

    for (int i = 0; i < 3; i++) {
	length += v[i] * v[i];
    }
    length = sqrt(length);

    return v / length;
}

// ? t0 array?
export void hit_triangle(uniform float e_x[], uniform float e_y[], uniform float e_z[],
				uniform float dir_x[], uniform float dir_y[], uniform float dir_z[],
				uniform double t0, uniform double t1[],
				uniform double vertices[], uniform double invMat[], uniform double nm[],
				uniform int start, uniform int end, 
				uniform int fullRecord, uniform float record[], uniform int hit_flag[],
				uniform float texCoord_x[], uniform float texCoord_y[],
				uniform float norm_x[], uniform float norm_y[], uniform float norm_z[]) {
    int8 local;

    /* vertex: TRI_VEC_SIZE 72
       position 0 1 2
       normal   3 4 5
       tex      6 7
       material 8
     */

    foreach (ind = start ... end) {
	local = 1;

	Vector4 ray_e = {e_x[ind], e_y[ind], e_z[ind], 1.f};
	Vector4 ray_d = {dir_x[ind], dir_y[ind], dir_z[ind], 1.f};
	Vector3 e_obj;
	Vector3 d_obj;

	///////// transform
	e_obj.x = invMat[0] * ray_e.x + invMat[1] * ray_e.y + invMat[2] * ray_e.z + invMat[2] * ray_e.w;
	e_obj.y = invMat[4] * ray_e.x + invMat[5] * ray_e.y + invMat[6] * ray_e.z + invMat[7] * ray_e.w;
	e_obj.z = invMat[8] * ray_e.x + invMat[9] * ray_e.y + invMat[10] * ray_e.z + invMat[11] * ray_e.w;
	float e_obj_w = invMat[12] * ray_e.x + invMat[13] * ray_e.y + invMat[14] * ray_e.z + invMat[15] * ray_e.w;

	e_obj.x = e_obj.x / e_obj_w;
	e_obj.y = e_obj.y / e_obj_w;
	e_obj.z = e_obj.z / e_obj_w;

	d_obj.x = nm[0] * ray_d.x + nm[1] * ray_d.y + nm[2] * ray_d.z + nm[2] * ray_d.w;
	d_obj.y = nm[4] * ray_d.x + nm[5] * ray_d.y + nm[6] * ray_d.z + nm[7] * ray_d.w;
	d_obj.z = nm[8] * ray_d.x + nm[9] * ray_d.y + nm[10] * ray_d.z + nm[11] * ray_d.w;
	float d_obj_w = nm[12] * ray_d.x + nm[13] * ray_d.y + nm[14] * ray_d.z + nm[15] * ray_d.w;

	d_obj.x = d_obj.x / d_obj_w;
	d_obj.y = d_obj.y / d_obj_w;
	d_obj.z = d_obj.z / d_obj_w;

	//////////////

	Vector3 v_0 = { vertices[0], vertices[1], vertices[2] };
	Vector3 v_1 = { vertices[TRI_VEC_SIZE], vertices[TRI_VEC_SIZE + 1], vertices[TRI_VEC_SIZE + 2] };
	Vector3 v_2 = { vertices[2 * TRI_VEC_SIZE], vertices[2 * TRI_VEC_SIZE + 1],
			vertices[2 * TRI_VEC_SIZE + 2] };

	Vector3 a_minus_b = v_0 - v_1;
	Vector3 a_minus_c = v_0 - v_2;
	Vector3 a_minus_e = v_0 - e_obj;

	float a = a_minus_b.x;
	float b = a_minus_b.y;
	float c = a_minus_b.z;
	float d = a_minus_c.x;
	float e = a_minus_c.y;
	float f = a_minus_c.z;
	float g = d_obj.x;
	float h = d_obj.y;
	float i = d_obj.z;
	float j = a_minus_e.x;
	float k = a_minus_e.y;
	float l = a_minus_e.z;

	float ei_minus_hf = e * i - h * f;
	float gf_minus_di = g * f - d * i;
	float dh_minus_eg = d * h - e * g;
	float ak_minus_jb = a * k - j * b;
	float jc_minus_al = j * c - a * l;
	float bl_minus_kc = b * l - k * c;

	float M = a * ei_minus_hf + b * gf_minus_di + c * dh_minus_eg;
	float time = (f * ak_minus_jb + e * jc_minus_al + d * bl_minus_kc) / -M;
	if (time <= t0 || time >= t1[ind]) {
	    local = 0;
	}

	if (!local) continue;
	float beta = (j * ei_minus_hf + k * gf_minus_di + l * dh_minus_eg) / M;
	if (beta < 0 || beta > 1)
	    local &= 0;
	if (!local) continue;
	float gamma = (i * ak_minus_jb + h * jc_minus_al + g * bl_minus_kc) / M;
	if (gamma < 0 || gamma > 1 - beta)
	    local &= 0;
	if (!local) continue;

	record[ind] = time;
	hit_flag[ind] |= local;

	if (!fullRecord)
	    continue;

	/////////// t value ends /// texture
	float alpha = 1 - beta - gamma;
	Vector2 tex = {0.f, 0.f};
	
	for (int count = 0; count < 2; count++) {
	    tex[count] = alpha * vertices[count + 6] + beta * vertices[count + 6 + TRI_VEC_SIZE] + 
		gamma * vertices[count + 6 + 2 * TRI_VEC_SIZE];
	    tex[count] -= (int) tex[count];
	}
	texCoord_x[ind] = tex[0];
	texCoord_y[ind] = tex[1];

	////////// norm
	Vector3 n;

	for (int count = 0; count < 3; count++) {
	    n[count] = alpha * vertices[3 + count] + beta * vertices[3 + TRI_VEC_SIZE + count] + 
		gamma * vertices[3 + 2 * TRI_VEC_SIZE + count];
	}
	
	n = normalize(n);

	norm_x[ind] = n[0];
	norm_y[ind] = n[1];
	norm_z[ind] = n[2];
    }
}

export uniform int hit(uniform float e_x[], uniform float e_y[], uniform float e_z[],
		       uniform float dir_x[], uniform float dir_y[], uniform float dir_z[],
		       uniform float t0, uniform float t1, 
		       uniform double lowCoord[], uniform double highCoord[],
		       uniform int start, uniform int end, uniform int fullRecord, uniform int8 result[]) 
{
    int<3> dirIsNeg;
    uniform int continue_flag = 1;
    uniform int minHit = BIG_NUMBER;
    int local = 1;
    for (uniform int ind = start; ind<end ; ind+=programCount) 
    {
        int i = ind + programIndex;
        if(i>=end)
            break;
        Vector3 e = {e_x[i], e_y[i], e_z[i]};
        Vector3 invDir = {rcp(dir_x[i]), rcp(dir_y[i]), rcp(dir_z[i])};
        dirIsNeg = invDir < 0;

        float tmin = (dirIsNeg[0] *  highCoord[0]) + (1 - dirIsNeg[0]) * lowCoord[0];
        tmin = (tmin - e[0]) * invDir[0];
        float tmax = (dirIsNeg[0] *  lowCoord[0]) + (1 - dirIsNeg[0]) * highCoord[0];
        tmax = (tmax - e[0]) * invDir[0];

        float tymin = (dirIsNeg[1] *  highCoord[1]) + (1 - dirIsNeg[1]) * lowCoord[1];
        tymin = (tymin - e[1]) * invDir[1];
        float tymax = (dirIsNeg[1] *  lowCoord[1]) + (1 - dirIsNeg[1]) * highCoord[1];
        tymax = (tymax - e[1]) * invDir[1];

        local = !((tmin > tymax) || (tymin > tmax));

	if (!local) continue;

        if (tymin > tmin) tmin = tymin;
        if (tymax < tmax) tmax = tymax;

        float tzmin = (dirIsNeg[2] *  highCoord[2]) + (1 - dirIsNeg[2]) * lowCoord[2];
        tzmin = (tzmin - e[2]) * invDir[2];
        float tzmax = (dirIsNeg[2] *  lowCoord[2]) + (1 - dirIsNeg[2]) * highCoord[2];
        tzmax = (tzmax - e[2]) * invDir[2];

        local &= ! ((tmin > tzmax) || (tzmin > tmax));
	if (!local) continue;

        // Because the cost after this is low, we don't check local flag?
        
        if (tzmin > tmin)   tmin = tzmin;
        if (tzmax < tmax)   tmax = tzmax;

        local &= !( tmax<t0 || tmin>t1 );
        local &= (tmin <= tmax + 1e-5);
	if (!local) continue;

        //if(ind ==192 && programIndex ==0)
        //print("\nbh:%\n %\n %\n %\n %\n %\n %\n", ind, programIndex, tmin, tmax, local, tzmin, tzmax);//, e[0], e[1], e[2], invDir[0], invDir[1], invDir[2]);
            
	if (!fullRecord) {
	    int gen = i + (1-local)*BIG_NUMBER;
	    minHit = reduce_min(gen);
	    if(minHit<end)
		break;
	}
	else {
	    result[i] = local;
	    if (local == 1)
		minHit = 0;
	}
    }
    if(minHit<end)
        return minHit;
    else
        return end;
}

export uniform int hitLast(uniform float e_x[], uniform float e_y[], uniform float e_z[],
         uniform float dir_x[], uniform float dir_y[], uniform float dir_z[],
         uniform float t0, uniform float t1, 
         uniform double lowCoord[], uniform double highCoord[],
         uniform int start, uniform int end) 
{
    int<3> dirIsNeg;
    uniform int continue_flag = 1;
    uniform int maxHit = -1;
    int local = 1;
    for (uniform int ind = end - programCount; ind > start - programCount; ind -= programCount) 
    {
        int i = ind + programIndex;
        if(i < start)
            break;
        Vector3 e = {e_x[i], e_y[i], e_z[i]};
        Vector3 invDir = {rcp(dir_x[i]), rcp(dir_y[i]), rcp(dir_z[i])};
        dirIsNeg = invDir < 0;

        float tmin = (dirIsNeg[0] *  highCoord[0]) + (1 - dirIsNeg[0]) * lowCoord[0];
        tmin = (tmin - e[0]) * invDir[0];
        float tmax = (dirIsNeg[0] *  lowCoord[0]) + (1 - dirIsNeg[0]) * highCoord[0];
        tmax = (tmax - e[0]) * invDir[0];

        float tymin = (dirIsNeg[1] *  highCoord[1]) + (1 - dirIsNeg[1]) * lowCoord[1];
        tymin = (tymin - e[1]) * invDir[1];
        float tymax = (dirIsNeg[1] *  lowCoord[1]) + (1 - dirIsNeg[1]) * highCoord[1];
        tymax = (tymax - e[1]) * invDir[1];

        local = !((tmin > tymax) || (tymin > tmax));

	if (!local) continue;

        if (tymin > tmin) tmin = tymin;
        if (tymax < tmax) tmax = tymax;

        float tzmin = (dirIsNeg[2] *  highCoord[2]) + (1 - dirIsNeg[2]) * lowCoord[2];
        tzmin = (tzmin - e[2]) * invDir[2];
        float tzmax = (dirIsNeg[2] *  lowCoord[2]) + (1 - dirIsNeg[2]) * highCoord[2];
        tzmax = (tzmax - e[2]) * invDir[2];

        local &= ! ((tmin > tzmax) || (tzmin > tmax));
	if (!local) continue;
        // Because the cost after this is low, we don't check local flag?
        
        if (tzmin > tmin)   tmin = tzmin;
        if (tzmax < tmax)   tmax = tzmax;

        local &= !( tmax<t0 || tmin>t1 );
        local &= (tmin <= tmax + 1e-5);
	if (!local) continue;

        //if(ind ==192 && programIndex ==0)
        //print("\nbh:%\n %\n %\n %\n %\n %\n %\n", ind, programIndex, tmin, tmax, local, tzmin, tzmax);//, e[0], e[1], e[2], invDir[0], invDir[1], invDir[2]);
            
        int gen = local * i + (1 - local) * (-1);
        maxHit = reduce_max(gen);
        if(maxHit > 0)
            break;
    }
    if(maxHit >= start)
        return maxHit + 1;
    else
        return start;
}
