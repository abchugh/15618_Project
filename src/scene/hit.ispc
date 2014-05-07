
typedef float<3> Vector3;

export uniform int hit(uniform float e_x[], uniform float e_y[], uniform float e_z[],
         uniform float dir_x[], uniform float dir_y[], uniform float dir_z[],
         uniform float t0, uniform float t1, 
         uniform float lowCoord[], uniform float highCoord[],
         uniform int start, uniform int end) 
{
    int<3> dirIsNeg;
    uniform int continue_flag = 1;
    uniform int minHit = 1000000;
    int local = 1;
    for (uniform int ind = start; ind<end ; ind+=programCount) 
    {
        int i = ind + programIndex;
        if(i>=end)
            break;
        Vector3 e = {e_x[i], e_y[i], e_z[i]};
        Vector3 invDir = {1/dir_x[i], 1/dir_y[i], 1/dir_z[i]};
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

        // TODO: maybe expensive
        continue_flag = reduce_add(local);
        //if(ind ==192 && programIndex ==0)
        //print("%\n %\n %\n %\n %\n %\n %\n %\n %\n %\n\n", tmin, tmax, tymin, tymax, e[0], e[1], e[2], invDir[0], invDir[1], invDir[2]);
            
        if (!continue_flag)
        {
            continue;
        }

        if (tymin > tmin) tmin = tymin;
        if (tymax < tmax) tmax = tymax;

        float tzmin = (dirIsNeg[2] *  highCoord[2]) + (1 - dirIsNeg[2]) * lowCoord[2];
        tzmin = (tzmin - e[2]) * invDir[2];
        float tzmax = (dirIsNeg[2] *  lowCoord[2]) + (1 - dirIsNeg[2]) * highCoord[2];
        tzmax = (tzmax - e[2]) * invDir[2];

        local &= ! ((tmin > tzmax) || (tzmin > tmax));

        // Because the cost after this is low, we don't check local flag?
        
        if (tzmin > tmin)   tmin = tzmin;
        if (tzmax < tmax)   tmax = tzmax;

        local &= !( tmax<t0 || tmin>t1 );
        local &= (tmin <= tmax + 1e-5);

        //if(ind ==192 && programIndex ==0)
        //print("\nbh:%\n %\n %\n %\n %\n %\n %\n", ind, programIndex, tmin, tmax, local, tzmin, tzmax);//, e[0], e[1], e[2], invDir[0], invDir[1], invDir[2]);
            
        int gen = i + (1-local)*1000000;
        minHit = reduce_min(gen);
        if(minHit<10000)
            break;
    }
    if(minHit<end)
        return minHit;
    else
        return end;
}
