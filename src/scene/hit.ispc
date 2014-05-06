
typedef float<3> Vector3;

void hit(uniform float e_x[], uniform float e_y[], uniform float e_z[],
         uniform float invDir_x[], uniform float invDir_y[], uniform float invDir_z[],
         uniform float t0[], uniform float t1[], 
         uniform float lowCoord[], uniform float highCoord[],
         uniform int start, uniform int end, uniform int8 result[]) 
{
    int<3> dirIsNeg;
    uniform int continue_flag = 1;
    int local = 1;

    foreach (i = start ... end) 
    {
        Vector3 e = {e_x[i], e_y[i], e_z[i]};
        Vector3 invDir = {invDir_x[i], invDir_y[i], invDir_z[i]};
        dirIsNeg = invDir < 0;

        float tmin = (dirIsNeg[0] *  highCoord[0]) + (1 - dirIsNeg[0]) * lowCoord[0];
        tmin = (tmin - e[0]) * invDir[0];
        float tmax = (dirIsNeg[0] *  lowCoord[0]) + (1 - dirIsNeg[0]) * highCoord[0];
        tmax = (tmax - e[0]) * invDir[0];

        float tymin = (dirIsNeg[1] *  highCoord[1]) + (1 - dirIsNeg[1]) * lowCoord[1];
        tymin = (tymin - e[1]) * invDir[1];
        float tymax = (dirIsNeg[1] *  lowCoord[1]) + (1 - dirIsNeg[1]) * highCoord[1];
        tymax = (tymax - e[1]) * invDir[1];

        local = (tmin > tymax) || (tymin > tmax);

        // TODO: maybe expensive
        continue_flag = reduce_add(local);
        if (!continue_flag) {
            result[i] = local;
            continue;
        }

        tmin = max(tmin, tymin);
        tmax = min(tmax, tymax);

        float tzmin = (dirIsNeg[2] *  highCoord[2]) + (1 - dirIsNeg[2]) * lowCoord[2];
        tzmin = (tzmin - e[2]) * invDir[2];
        float tzmax = (dirIsNeg[2] *  lowCoord[2]) + (1 - dirIsNeg[2]) * highCoord[2];
        tzmax = (tzmax - e[2]) * invDir[2];

        local &= (tmin > tzmax) || (tzmin > tmax);

        // Because the cost after this is low, we don't check local flag?

        tmin = max(tmin, tzmin);
        tmax = min(tmax, tzmax);

        local &= (tmax < t0[i]) || (tmin > t1[i]);
        local &= (tmin <= tmax + 1e-5);

        result[i] = local;
    }
}
