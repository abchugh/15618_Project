
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
                         uniform double v0[], uniform double v1[], uniform double v2[],
                         uniform const double invMat[][4],
                         uniform int start, uniform int end, 
                         uniform int fullRecord, uniform int hit_flag[],
                         uniform float texCoord_x[], uniform float texCoord_y[],
                         uniform float norm_x[], uniform float norm_y[], uniform float norm_z[]) 
{
    int8 local;

    /* vertex: TRI_VEC_SIZE 72
    position 0 1 2
    normal   3 4 5
    tex      6 7
    material 8
    */
    
    uniform Vector3 v_0 = { v0[0], v0[1], v0[2] };
    uniform Vector3 v_1 = { v1[0], v1[1], v1[2] };
    uniform Vector3 v_2 = { v2[0], v2[1], v2[2] };

    uniform float a = v0[0] - v1[0];
    uniform float b = v0[1] - v1[1];
    uniform float c = v0[2] - v1[2];

    uniform float d = v0[0] - v2[0];
    uniform float e = v0[1] - v2[1];
    uniform float f = v0[2] - v2[2];

    foreach (ind = start ... end) {
        int index = ind - start;
        local = 1;

        Vector3 ray_e = {e_x[ind], e_y[ind], e_z[ind]};
        Vector3 ray_d = {dir_x[ind], dir_y[ind], dir_z[ind]};

        Vector3 e_obj;
        Vector3 d_obj;
        float e_obj_w;
        float d_obj_w;

        ///////// transform
        e_obj.x = invMat[0][0] * ray_e.x + invMat[1][0] * ray_e.y + invMat[2][0] * ray_e.z + invMat[3][0];
        e_obj.y = invMat[0][1] * ray_e.x + invMat[1][1] * ray_e.y + invMat[2][1] * ray_e.z + invMat[3][1];
        e_obj.z = invMat[0][2] * ray_e.x + invMat[1][2] * ray_e.y + invMat[2][2] * ray_e.z + invMat[3][2];
        e_obj_w = invMat[0][3] * ray_e.x + invMat[1][3] * ray_e.y + invMat[2][3] * ray_e.z + invMat[3][3];

        if( e_obj_w == 0)
            e_obj_w = 1;
        else
            e_obj_w = 1/e_obj_w;

        e_obj.x = e_obj.x * e_obj_w;
        e_obj.y = e_obj.y * e_obj_w;
        e_obj.z = e_obj.z * e_obj_w;
        
        /*m(3,0)=0;
	    m(3,1)=0;
	    m(3,2)=0;*/

        d_obj.x = invMat[0][0] * ray_d.x + invMat[1][0] * ray_d.y + invMat[2][0] * ray_d.z;
        d_obj.y = invMat[0][1] * ray_d.x + invMat[1][1] * ray_d.y + invMat[2][1] * ray_d.z;
        d_obj.z = invMat[0][2] * ray_d.x + invMat[1][2] * ray_d.y + invMat[2][2] * ray_d.z;
        d_obj_w = invMat[0][3] * ray_d.x + invMat[1][3] * ray_d.y + invMat[2][3] * ray_d.z + invMat[3][3];
        
        if( d_obj_w == 0)
            d_obj_w = 1;
        else
            d_obj_w = 1/d_obj_w;

        d_obj.x = d_obj.x * d_obj_w;
        d_obj.y = d_obj.y * d_obj_w;
        d_obj.z = d_obj.z * d_obj_w;
        
        //////////////


        float g = d_obj.x;
        float h = d_obj.y;
        float i = d_obj.z;

        float j = v0[0] - e_obj[0];
        float k = v0[1] - e_obj[1];
        float l = v0[2] - e_obj[2];

        float ei_minus_hf = e * i - h * f;
        float gf_minus_di = g * f - d * i;
        float dh_minus_eg = d * h - e * g;
        float ak_minus_jb = a * k - j * b;
        float jc_minus_al = j * c - a * l;
        float bl_minus_kc = b * l - k * c;

        float M = a * ei_minus_hf + b * gf_minus_di + c * dh_minus_eg;
        float time = (f * ak_minus_jb + e * jc_minus_al + d * bl_minus_kc) / -M;

        hit_flag[index] = 0;

        if (time <= t0 || time >= t1[ind]) 
            continue;
        

        float beta = (j * ei_minus_hf + k * gf_minus_di + l * dh_minus_eg) / M;
        if (beta < 0 || beta > 1)
            continue;
        
        float gamma = (i * ak_minus_jb + h * jc_minus_al + g * bl_minus_kc) / M;
        if (gamma < 0 || gamma > 1 - beta)
            continue;

        t1[ind] = time;

        hit_flag[index] = 1;

        if (!fullRecord)
            continue;

        /////////// t value ends /// texture
        float alpha = 1 - beta - gamma;
        Vector2 tex = {0.f, 0.f};

        for (int count = 0; count < 2; count++) {
            tex[count] = alpha * v0[count + 6] + beta * v1[count + 6] + gamma * v2[count + 6];
            tex[count] -= (int) tex[count];
        }
        texCoord_x[index] = tex[0];
        texCoord_y[index] = tex[1];

        ////////// norm
        Vector3 n;

        for (int count = 0; count < 3; count++) {
            n[count] = alpha * v0[3 + count] + beta * v1[3 + count] + gamma * v2[3 + count];
        }

        n = normalize(n);

        norm_x[index] = n[0];
        norm_y[index] = n[1];
        norm_z[index] = n[2];
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
