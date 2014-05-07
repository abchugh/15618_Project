#include "bvh.hpp"
#include "math/vector.hpp"
#include "scene/scene.hpp"
#include <SDL_timer.h>
#include <queue>
#include <omp.h>

#include <map>

//#define printf(...) 
using namespace std;

namespace _462 {

#ifdef ENABLED_TIME_LOGS
    time_t t1[MAX_THREADS], t2[MAX_THREADS], t3[MAX_THREADS], t4[MAX_THREADS], t5[MAX_THREADS], t6[MAX_THREADS], t7[MAX_THREADS], t8[MAX_THREADS], tP[MAX_THREADS];

    time_t _prevTick[MAX_THREADS];

#define AddTimeSincePreviousTick(interval)\
    {int tid = omp_get_thread_num();\
    time_t timeNew = SDL_GetTicks();\
    interval[tid] += timeNew - _prevTick[tid];\
    _prevTick[tid] = timeNew;}

#define GetTime(timeNew) time_t timeNew = SDL_GetTicks();\
    _prevTick[omp_get_thread_num()] = timeNew;

    static time_t sum(time_t arr[])
    {
        time_t sum = 0;
        for(int i=0;i<MAX_THREADS;i++)
            sum+=arr[i];
        return sum;
    }
    static void reset()
    {
        for(int i=0;i<MAX_THREADS;i++)
            t1[i] = t2[i] = t3[i] = t4[i] = t5[i] = t6[i] = t7[i] = t8[i] = tP[i] = 0;
    }
#else
#define AddTimeSincePreviousTick(interval)
#define GetTime(timeNew)
#endif

    void deleteRecursive( BVHBuildNode* node)
    {
        if(node->children[0])
            deleteRecursive(node->children[0]);
        if(node->children[1])
            deleteRecursive(node->children[1]);
        delete node;
    }

    void initPrimitiveInfoList(const std::vector<Geometry*>& primitives, PrimitiveInfoList& list, bool allocateOnly = false);
    void AddBox(const PrimitiveInfoList& buildData, uint32_t start, uint32_t end, BoundingBox & box);
    void AddCentroid(const PrimitiveInfoList& buildData, uint32_t start, uint32_t end, BoundingBox & box);

    void AddBox(const PrimitiveInfoList& buildData, uint32_t index, BoundingBox & box);

    float getCentroidDim(const PrimitiveInfoList& buildData, int index, int dim);
    uint32_t SplitEqually(PrimitiveInfoList& buildData, uint32_t start, uint32_t end, uint32_t dim);
    void clearList(PrimitiveInfoList& buildData);
    unsigned int partition(int start, int end, int dim, float mid, PrimitiveInfoList& buildData);

    bool queueData::updateStatus(queueData& data)
    {
        bool isValid = (*data.status==USE_AND_DELETE || *data.status==USE_AND_DONT_DELETE);
        if(*data.status==USE_AND_DELETE || *data.status==DONT_USE_AND_DELETE) {
            delete data.status;
            data.status = NULL;
        }
        else
            *data.status = DONT_USE_AND_DELETE;
        return isValid;
    }

    BVHAccel::BVHAccel(const vector<Geometry*>& geometries, uint32_t mp, const string &sm):nodes(NULL), root(NULL)
    {
        time_t startTime = SDL_GetTicks();

        printf("Building BVH...\n");
        maxPrimsInNode = min(255u, mp);
        primitives = geometries;

        if (primitives.size() == 0)
            return;

        // Initialize _buildData_ array for primitives
        PrimitiveInfoList buildData;
        initPrimitiveInfoList(primitives, buildData);

        uint32_t totalNodes = 0;
        vector< Geometry* > orderedPrims(primitives.size());

        queueData rootData = {0, static_cast<uint32_t>(primitives.size()),NULL, BoundingBox(), true, NULL };
        rootData.status = new char;
        *rootData.status = USE_AND_DELETE;
        pq.push(rootData);

        time_t endTime = SDL_GetTicks();
		omp_set_num_threads(12);
        int thread_count = omp_get_max_threads();

        for (int i = 0; i < thread_count; i++) {
            int block_size = (10 > primitives.size() / 5) ? 10 : primitives.size() / 5;
            int inc_size = (10 > primitives.size() / 10) ? 10 : primitives.size() / 10;
            poolPtr[i] = new BuildNodePool(block_size, inc_size);
        }
        poolPtr[thread_count] = new BuildNodePool(10, 10);
		
        printf("Started parallel node phase at %ld \n", endTime-startTime);
        while(pq.size()<=NUM_IN_PRE_QUEUE)//omp_get_max_threads())
        {
            queueData data = pq.top();
            if(data.end-data.start<=100)break;
            bool isValid = queueData::updateStatus(data);
            assert(isValid);
            pq.pop();
            BoundingBox *boxPtr = (data.parent == NULL) ? NULL : &data.box;
            BVHBuildNode *node = fastRecursiveBuild(buildData, data.start, data.end, boxPtr,
                &totalNodes, orderedPrims, 
                data.parent, data.isFirstChild);
            if(data.parent == NULL)
                root = node;
        }

        int working = 0;

        endTime = SDL_GetTicks();
        printf("Started parallel tree phase  at %ld \n", endTime-startTime);
        time_t busy[MAX_THREADS] = {0}, idle[MAX_THREADS] = {0}, idleX[MAX_THREADS] = {0};
#ifdef ENABLED_TIME_LOGS
        printf("\tLargeBB\tCent\tEquSpl\tBucket\tCost\tNode\tPart\tEnqueue\tLeaves\n");
        printf("Phases\t%ld\t%ld\t%ld\t%ld\t%ld\t%ld\t%ld\t%ld\t%ld\n", sum(t1), sum(t2), sum(t3), sum(t4), sum(t5), sum(t6), sum(t7), sum(t8), sum(tP));
        reset();	
#endif
        printf("omp_maxThreads: %d\n\n", omp_get_max_threads());
#pragma omp parallel num_threads(thread_count)
        {

            int tid = omp_get_thread_num();
            int backoff = 2;
            while(1)
            {
                time_t idleStart = SDL_GetTicks();
                queueData data;
                bool foundWork = false;

#pragma omp critical(queueUpdate)
                {
                    while(!q[tid].empty())
                    {
                        data = q[tid].back();
                        q[tid].pop_back();

                        bool isValid = queueData::updateStatus(data);

                        if(isValid)
                        {
                            working |= (1<<tid);
                            foundWork = true;
                            break;
                        }
                    }
                    if(!foundWork) while(!pq.empty())
                    {    
                        data = pq.top();    
                        pq.pop();
                        bool isValid = queueData::updateStatus(data);
                        if(isValid)
                        {
                            working |= (1<<tid);
                            foundWork = true;
                            break;
                        }
                    }
                    if(!foundWork)
                        working &= 0xFFFF - (1<<tid);
                }
                if(!foundWork)
                {    
                    SDL_Delay(backoff);
                    if(backoff<4)backoff*=2;
                }

                if(working == 0) //no one is working, time to end
                    break;

                time_t idleEnd = SDL_GetTicks();

                idleX[tid] += idleEnd-idleStart;
                if(!foundWork)
                    idle[tid] += idleEnd - idleStart;

                if(foundWork)
                {
                    time_t startT = SDL_GetTicks();
                    BoundingBox *boxPtr = (data.parent == NULL) ? NULL : &data.box;
                    BVHBuildNode* node = recursiveBuild(buildData, data.start, data.end, boxPtr,
                        &totalNodes, orderedPrims, 
                        data.parent, data.isFirstChild);
                    if(data.parent == NULL)
                        root = node;
                    time_t endT = SDL_GetTicks();
                    busy[tid]+=endT-startT;
                }
            }
        }

        assert(root!=NULL);
        primitives.swap(orderedPrims);

        endTime = SDL_GetTicks();

        printf("Ended parallel tree phase at %ld \n", endTime-startTime);
#ifdef ENABLED_TIME_LOGS
        printf("\tLargeBB\tCent\tEquSpl\tBucket\tCost\tNode\tPart\tEnqueue\tLeaves\n");
        printf("Phases\t%ld\t%ld\t%ld\t%ld\t%ld\t%ld\t%ld\t%ld\t%ld\n", sum(t1), sum(t2), sum(t3), sum(t4), sum(t5), sum(t6), sum(t7), sum(t8), sum(tP));
#endif
        for(int i=0;i<MAX_THREADS;i++) if(busy[i]!=0)
            printf("%ld ", busy[i]);
        printf("\n");
        for(int i=0;i<MAX_THREADS;i++) if(busy[i]!=0)
            printf("%ld/%ld ", idle[i], idleX[i]);
        printf("%d\n",totalNodes);

        // Compute representation of depth-first traversal of BVH tree
        nodes = new LinearBVHNode[totalNodes];

        uint32_t offset = 0;
        flattenBVHTree(root, &offset);
        assert(offset == totalNodes);
        endTime = SDL_GetTicks();

        clearList(buildData);

        printf("Done Building BVH at %ld \n\n", endTime-startTime);
    }

    BVHAccel::~BVHAccel() {
        int thread_count = omp_get_max_threads();

        for (int i = 0; i < thread_count + 1; i++) {
            poolPtr[i]->destroy();
            delete (poolPtr[i]);
        }
        
        /*if(root)
        {
            deleteRecursive(root);
            root = NULL;
        }*/
        
        if(nodes)
        {
            delete []nodes;
            nodes = NULL;
        }
    }

    //TODO:convert into #define to check for perf improvement?
    void BVHAccel::buildLeaf(PrimitiveInfoList &buildData, uint32_t start,
        uint32_t end, vector<Geometry* > &orderedPrims, BVHBuildNode *node, const BoundingBox& bbox)
    {
        GetTime(primitiveStart);
        for (uint32_t i = start; i < end; ++i)
        {
            uint32_t primitiveNo;
#ifdef ISPC_SOA
            primitiveNo = buildData.primitiveNumber[i];
#elif defined(ISPC_AOS)
            memcpy(&primitiveNo, &(buildData[i].lowCoord.v[3]), sizeof(uint32_t));
#else
            primitiveNo = buildData[i].primitiveNumber;
#endif
            orderedPrims[i] = primitives[ primitiveNo ];
        }
        node->InitLeaf(start, end-start, bbox);

        AddTimeSincePreviousTick(tP);
    }

    BVHBuildNode *BVHAccel::recursiveBuild( PrimitiveInfoList &buildData, uint32_t start,
        uint32_t end, BoundingBox *boxPtr, uint32_t *totalNodes, 
        vector<Geometry* > &orderedPrims, BVHBuildNode *parent, bool firstChild) {
            if (start == end)
                printf("%d %d\n", start, end);
            assert(start != end);
            GetTime(startTime);

#pragma omp atomic
            (*totalNodes)++;

            //BVHBuildNode *node = new BVHBuildNode(parent, firstChild);
            int tid = (uint32_t)omp_get_thread_num();
            BVHBuildNode *node = poolPtr[tid]->allocate(parent, firstChild);
            if(parent)
                parent->children[firstChild?0:1] = node;

            int numInternalBranches = 0;
            queueData child1Data, child2Data;

            uint32_t nPrimitives = end - start;
            AddTimeSincePreviousTick(t6);

            // Compute bounds of all primitives in BVH node
            BoundingBox *bboxPtr;
            if (boxPtr == NULL) {
                // Potentially rounding problem.
                BoundingBox newBbox;
                AddBox(buildData, start, end, newBbox);
                bboxPtr = &newBbox;
            }
            else {
                bboxPtr = boxPtr;
                assert(bboxPtr->extent(0) >= 0);
            }

            AddTimeSincePreviousTick(t1);

            if (nPrimitives == 1) {
                buildLeaf(buildData,start,end, orderedPrims,node,*bboxPtr);
            }
            else {
                // Compute bound of primitive centroids, choose split dimension _dim_

                BoundingBox centroidBounds;
                AddCentroid(buildData, start, end, centroidBounds);

                int dim = centroidBounds.MaximumExtent();

                //int dim = bbox.MaximumExtent();
                AddTimeSincePreviousTick(t2);

                // Partition primitives into two sets and build children
                uint32_t mid = (start + end) / 2;
                // Change == to < to fix bug. There might be case that low is too close to high that partition
                // can't really separate them.
                if (centroidBounds.extent(dim) < 1e-5) {
                    buildLeaf(buildData,start,end, orderedPrims,node,*bboxPtr);
                    goto finishUp;
                }

                // Partition primitives using approximate SAH
                if (nPrimitives <= 4) {
                    // Partition primitives into equally-sized subsets
                    mid = SplitEqually(buildData, start, end, dim);

                    // build aabbs for children
                    BoundingBox b0, b1;
                    AddBox(buildData, start, mid, b0);
                    AddBox(buildData, mid, end, b1);

                    child1Data.box = b0;
                    child2Data.box = b1;

                    AddTimeSincePreviousTick(t3);
                }
                else {
                    // Allocate _BucketInfo_ for SAH partition buckets
                    const int nBuckets = 12;
                    struct BucketInfo {
                        BucketInfo() { count = 0; }
                        int count;
                        BoundingBox bounds;
                    };
                    BucketInfo buckets[nBuckets];

                    // Initialize _BucketInfo_ for SAH partition buckets
                    for (uint32_t j = 0; j < nPrimitives; ++j) {
                        uint32_t i = j + start;

                        float val = getCentroidDim(buildData, i, dim);

                        int b = nBuckets *
                            ( (val - centroidBounds.lowCoord[dim]) / centroidBounds.extent(dim));
                        if (b == nBuckets) b = nBuckets-1;

                        assert(b >= 0 && b < nBuckets);
                        buckets[b].count++;
                        AddBox(buildData, i, buckets[b].bounds);
                    }
                    AddTimeSincePreviousTick(t4);

                    // Compute costs for splitting after each bucket
                    float minCost = BIG_NUMBER;
                    uint32_t minCostSplit = 0;
                    BucketInfo inc_buckets[nBuckets - 1];
                    BucketInfo dec_buckets[nBuckets - 1];

                    for (uint32_t i = 0; i < nBuckets - 1; i++) {
                        BoundingBox prev_inc;
                        BoundingBox prev_dec;
                        prev_inc = (i > 0) ? inc_buckets[i - 1].bounds : prev_inc;
                        prev_dec = (i > 0) ? dec_buckets[i - 1].bounds : prev_dec;

                        int prev_inc_count = (i > 0) ? inc_buckets[i - 1].count : 0;
                        int prev_dec_count = (i > 0) ? dec_buckets[i - 1].count : 0;

                        inc_buckets[i].bounds = buckets[i].bounds;
                        inc_buckets[i].bounds.AddBox(prev_inc);
                        inc_buckets[i].count = buckets[i].count + prev_inc_count;

                        dec_buckets[i].bounds = buckets[nBuckets- 1 - i].bounds;
                        dec_buckets[i].bounds.AddBox(prev_dec);
                        dec_buckets[i].count = buckets[nBuckets- 1 - i].count + prev_dec_count;
                    }

                    for (int i = 0; i < nBuckets - 1; i++) {
                        BoundingBox b0, b1;
                        int count0 = 0, count1 = 0;

                        b0 = inc_buckets[i].bounds;
                        b1 = dec_buckets[nBuckets - 2 - i].bounds;
                        count0 = inc_buckets[i].count;
                        count1 = dec_buckets[nBuckets - 2 - i].count;
                        float cost = .125f + (count0 * b0.SurfaceArea() + count1 * b1.SurfaceArea()) /
                            bboxPtr->SurfaceArea();
                        if (cost < minCost) {
                            minCostSplit = i;
                            child1Data.box = b0;
                            child2Data.box = b1;
                            minCost = cost;
                        }
                    }

                    AddTimeSincePreviousTick(t5);

                    // Either create leaf or split primitives at selected SAH bucket
                    if (nPrimitives > maxPrimsInNode || minCost < nPrimitives) {
                        float bmid = (minCostSplit + 1) * centroidBounds.extent(dim) / nBuckets + centroidBounds.lowCoord[dim];
                        mid = partition(start, end, dim, bmid, buildData);

                        AddTimeSincePreviousTick(t7);
                    }
                    else {
                        buildLeaf(buildData,start,end, orderedPrims,node,*bboxPtr);
                        goto finishUp;
                    }
                }

                node->splitAxis = dim;
                node->bounds = *bboxPtr;

                numInternalBranches = (nPrimitives>200)?1:2;
                child1Data.start = start;
                child1Data.end = mid;
                child1Data.parent = node;
                child1Data.isFirstChild= true;

                child2Data.start = mid;
                child2Data.end = end;
                child2Data.parent = node;
                child2Data.isFirstChild = false;

                if(numInternalBranches==1)
                {            
                    child2Data.status = new char;
                    *child2Data.status = USE_AND_DELETE;
                    q[omp_get_thread_num()].push_back(child2Data);

                    queueData top;
#pragma omp critical(queueUpdate)
                    {
                        if((int)pq.size()<omp_get_max_threads())
                        {
                            *child2Data.status = USE_AND_DONT_DELETE;
                            pq.push(child2Data);
                        }
                        else
                        {
                            top = pq.top();
                            if( (top.end-top.start)/8< (child2Data.end-child2Data.start))
                            {
                                *child2Data.status = USE_AND_DONT_DELETE;
                                pq.push(child2Data);
                            }
                        }
                    }
                }
            }
finishUp:

            AddTimeSincePreviousTick(t8);
            if(numInternalBranches>=1)
            {
                recursiveBuild(buildData, child1Data.start, child1Data.end, &child1Data.box,
                    totalNodes, orderedPrims, child1Data.parent, child1Data.isFirstChild);
                if(numInternalBranches==2)
                {
                    recursiveBuild(buildData, child2Data.start, child2Data.end, &child2Data.box,
                        totalNodes, orderedPrims, child2Data.parent, child2Data.isFirstChild);
                }
            }

            return node;
    }

    BVHBuildNode *BVHAccel::fastRecursiveBuild( PrimitiveInfoList &buildData, uint32_t start,
        uint32_t end, BoundingBox *boxPtr, uint32_t *totalNodes,
        vector<Geometry* > &orderedPrims, BVHBuildNode *parent, bool firstChild) {
            assert(end-start>100);

            GetTime(startTime);

            (*totalNodes)++;

            //BVHBuildNode *node = new BVHBuildNode(parent, firstChild);
            int thread_count = omp_get_max_threads();
            BVHBuildNode *node = poolPtr[thread_count]->allocate(parent, firstChild);
            if(parent)
                parent->children[firstChild?0:1] = node;

            uint32_t nPrimitives = end - start;
            AddTimeSincePreviousTick(t6);

            // Compute bounds of all primitives in BVH node
            const int maxNumThreads = omp_get_max_threads();
            BoundingBox bbox;
            BoundingBox *subBbox = new BoundingBox[maxNumThreads];

            BoundingBox centroidBounds;
            BoundingBox *subCentroidBounds = new BoundingBox[maxNumThreads];

            const int nBuckets = 12;
            struct BucketInfo {
                BucketInfo() { count = 0; }
                int count;
                BoundingBox bounds;
            };
            BucketInfo buckets[nBuckets];

            BucketInfo **subBuckets = new BucketInfo*[maxNumThreads];
            for(int i=0;i<maxNumThreads;i++)
                subBuckets[i] = new BucketInfo[nBuckets];

            int dim=0;
#pragma omp parallel
            {

#ifdef CENTROID_BASED
                GetTime(startT);
                int threadNo = (uint32_t)omp_get_thread_num();
                uint32_t numPerThread = (end-start-1)/omp_get_num_threads();
                uint32_t s = start + threadNo*numPerThread;
                uint32_t e = (threadNo==omp_get_num_threads()-1)?end:(start + (threadNo+1)*numPerThread);

                AddCentroid(buildData, s, e, subCentroidBounds[threadNo]);

#pragma omp barrier
                AddTimeSincePreviousTick(t2);


#pragma omp single
                {
                    for(int i=0;i<maxNumThreads;i++)
                        centroidBounds.AddBox(subCentroidBounds[i]);
                    dim = centroidBounds.MaximumExtent();
                }
#else
                GetTime(startT);
                int threadNo = (uint32_t)omp_get_thread_num();
                uint32_t numPerThread = (end-start-1)/omp_get_num_threads();
                uint32_t s = start + threadNo*numPerThread;
                uint32_t e = (threadNo==omp_get_num_threads()-1)?end:(start + (threadNo+1)*numPerThread);

                if (boxPtr == NULL) {
                    AddBox(buildData, s, e, subCentroidBounds[threadNo]);

#pragma omp barrier

#pragma omp single
                    {
                        for(int i=0;i<maxNumThreads;i++)
                            bbox.AddBox(subCentroidBounds[i]);
                        dim = bbox.MaximumExtent();
                    }

                }
                else {
                    bbox = *boxPtr;
                    assert(bbox.extent(0) > 0);
                }
                AddTimeSincePreviousTick(t1);		
#endif
                for (uint32_t i = s; i < e; ++i)
                {                

                    float val = getCentroidDim(buildData, i, dim);

#ifdef CENTROID_BASED
                    int b = nBuckets *
                        ((val - centroidBounds.lowCoord[dim]) / centroidBounds.extent(dim));
#else
                    int b = nBuckets *
                        ((val - bbox.lowCoord[dim]) / bbox.extent(dim));
#endif
                    if (b == nBuckets) b = nBuckets-1;
                    assert(b >= 0 && b < nBuckets);
                    {
                        subBuckets[threadNo][b].count++;
                        AddBox(buildData, i, subBuckets[threadNo][b].bounds);
                    }
                }

                AddBox(buildData, s, e, subBbox[threadNo]);

#pragma omp barrier
#pragma omp single
                {
                    //can parallelize this. But is it worth the effort?
                    for(int i=0;i<maxNumThreads;i++)for(int b=0;b<nBuckets;b++) {
                        buckets[b].count +=  subBuckets[i][b].count;
                        buckets[b].bounds.AddBox(subBuckets[i][b].bounds);
                    }
                }
                AddTimeSincePreviousTick(t4);

#ifdef CENTROID_BASED		
                if (boxPtr == NULL) {
                    if(threadNo==0) 
                        for(int i=0;i<maxNumThreads;i++)
                            bbox.AddBox(subBbox[i]);
                }
                else
                    bbox = *boxPtr;		

                AddTimeSincePreviousTick(t1);
#endif
            }

            delete [] subBbox;
            delete [] subCentroidBounds;
            for(int i=0;i<maxNumThreads;i++)
                delete[] subBuckets[i];
            delete[] subBuckets;

            BoundingBox chb0, chb1;
            float minCost = BIG_NUMBER;
            uint32_t minCostSplit = 0;
            BucketInfo inc_buckets[nBuckets - 1];
            BucketInfo dec_buckets[nBuckets - 1];

            for (uint32_t i = 0; i < nBuckets - 1; i++) {
                BoundingBox prev_inc;
                BoundingBox prev_dec;
                prev_inc = (i > 0) ? inc_buckets[i - 1].bounds : prev_inc;
                prev_dec = (i > 0) ? dec_buckets[i - 1].bounds : prev_dec;

                int prev_inc_count = (i > 0) ? inc_buckets[i - 1].count : 0;
                int prev_dec_count = (i > 0) ? dec_buckets[i - 1].count : 0;

                inc_buckets[i].bounds = buckets[i].bounds;
                inc_buckets[i].bounds.AddBox(prev_inc);
                inc_buckets[i].count = buckets[i].count + prev_inc_count;

                dec_buckets[i].bounds = buckets[nBuckets- 1 - i].bounds;
                dec_buckets[i].bounds.AddBox(prev_dec);
                dec_buckets[i].count = buckets[nBuckets- 1 - i].count + prev_dec_count;
            }

            for (int i = 0; i < nBuckets - 1; i++) {
                BoundingBox b0, b1;
                int count0 = 0, count1 = 0;

                b0 = inc_buckets[i].bounds;
                b1 = dec_buckets[nBuckets - 2 - i].bounds;
                count0 = inc_buckets[i].count;
                count1 = dec_buckets[nBuckets - 2 - i].count;
                float cost = .125f + (count0 * b0.SurfaceArea() + count1 * b1.SurfaceArea()) /
                    bbox.SurfaceArea();
                if (cost < minCost) {
                    minCostSplit = i;
                    chb0 = b0;
                    chb1 = b1;
                    minCost = cost;
                }
            }

            AddTimeSincePreviousTick(t5);

            uint32_t mid = 0;
            // Either create leaf or split primitives at selected SAH bucket
            if (nPrimitives > maxPrimsInNode ||
                minCost < nPrimitives) {
                    //float bmid = (minCostSplit+1) * centroidBounds.extent(dim) / nBuckets + centroidBounds.lowCoord[dim];
                    float bmid = (minCostSplit+1) * bbox.extent(dim) / nBuckets + bbox.lowCoord[dim];
                    mid = partition(start, end, dim, bmid, buildData);

            }

            AddTimeSincePreviousTick(t7);

            node->splitAxis = dim;
            node->bounds = bbox;

            queueData child1Data = {start, mid, node, chb0, true, NULL};
            queueData child2Data = {mid,   end, node, chb1, false, NULL};
            child1Data.status = new char;
            *child1Data.status = USE_AND_DELETE;
            child2Data.status = new char;
            *child2Data.status = USE_AND_DELETE;

            if(child1Data<child2Data)
                swap(child1Data, child2Data);

            pq.push(child1Data);
            if(child2Data.end - child2Data.start > 100 && pq.size()<=NUM_IN_PRE_QUEUE)
                fastRecursiveBuild(buildData, child2Data.start, child2Data.end, &child2Data.box, totalNodes,
                orderedPrims, child2Data.parent, child2Data.isFirstChild);
            else
                pq.push(child2Data);

            AddTimeSincePreviousTick(t8);
            return node;
    }

    uint32_t BVHAccel::flattenBVHTree(BVHBuildNode *node, uint32_t *offset)
    {
        LinearBVHNode *linearNode = &nodes[*offset];
        linearNode->bounds = node->bounds;
        uint32_t myOffset = (*offset)++;
        if (node->nPrimitives > 0) {
            assert(!node->children[0] && !node->children[1]);
            linearNode->primitivesOffset = node->firstPrimOffset;
            linearNode->nPrimitives = node->nPrimitives;
        }
        else {
            // Creater interior flattened BVH node
            linearNode->axis = node->splitAxis;
            linearNode->nPrimitives = 0;
            flattenBVHTree(node->children[0], offset);
            linearNode->secondChildOffset = flattenBVHTree(node->children[1], offset);
        }
        return myOffset;
    }

    uint32_t BVHAccel::getFirstHit(const Packet& packet, const BoundingBox& box, uint32_t active,
        uint32_t *dirIsNeg, real_t t0, real_t t1, const vector<hitRecord>& records, bool fullRecord) const {
            Ray active_ray = packet.rays[active];
            Vector3 invDir(1.f / active_ray.d.x, 1.f / active_ray.d.y, 1.f / active_ray.d.z);
            dirIsNeg[0] = invDir.x < 0;
            dirIsNeg[1] = invDir.y < 0;
            dirIsNeg[2] = invDir.z < 0;

            if ( (fullRecord || records[active].t>=1 ) && box.hit(invDir, active_ray.e, t0, t1, dirIsNeg))
                return active;
            if (packet.frustum.isValid && !box.hit(packet.frustum))
                return packet.size;
            for (uint32_t i = active + 1; i < packet.size; i++) {
                Ray cur_ray = packet.rays[i];
                Vector3 invDir(1.f / cur_ray.d.x, 1.f / cur_ray.d.y, 1.f / cur_ray.d.z);
                dirIsNeg[0] = invDir.x < 0;
                dirIsNeg[1] = invDir.y < 0;
                dirIsNeg[2] = invDir.z < 0;

                if ( (fullRecord || records[i].t>=1 ) && box.hit(invDir, cur_ray.e, t0, t1, dirIsNeg))
                    return i;
            }

            return packet.size;
    }

    uint32_t BVHAccel::getLastHit(const Packet& packet, const BoundingBox& box, uint32_t active,
        uint32_t *dirIsNeg, real_t t0, real_t t1, const vector<hitRecord>& records, bool fullRecord) const {
            Ray active_ray = packet.rays[active];
            Vector3 invDir(1.f / active_ray.d.x, 1.f / active_ray.d.y, 1.f / active_ray.d.z);
            dirIsNeg[0] = invDir.x < 0;
            dirIsNeg[1] = invDir.y < 0;
            dirIsNeg[2] = invDir.z < 0;

            for (uint32_t i = packet.size - 1; i > active; i--) {
                Ray cur_ray = packet.rays[i];
                Vector3 invDir(1.f / cur_ray.d.x, 1.f / cur_ray.d.y, 1.f / cur_ray.d.z);
                dirIsNeg[0] = invDir.x < 0;
                dirIsNeg[1] = invDir.y < 0;
                dirIsNeg[2] = invDir.z < 0;

                if ( (fullRecord || records[i].t>=1 ) && box.hit(invDir, cur_ray.e, t0, t1, dirIsNeg))
                    return i+1;
            }

            return active+1;
    }


    void BVHAccel::hit(const Packet& packet, const real_t t0, const real_t t1, vector<hitRecord>& records, bool fullRecord) const
    {
        if(!nodes || packet.size==0)
            return ;

        TraversalNode stack[64]; // fixed size?
        uint32_t todoOffset = 0;
        uint32_t nodeNum = 0;
        uint32_t active = 0;

        uint32_t dirIsNeg[3];
        real_t t1_max = t1;

        for (uint32_t i = 0; i < packet.size; i++)
            records[i].t = t1*2;

        hitRecord h1;
        while (true) {
            const LinearBVHNode *node = &nodes[nodeNum];

            // TODO: a better way to get t1_max? Or do we actually need this?

            for (uint32_t i = active; i < packet.size; i++) {
                t1_max = std::max(t1_max, records[i].t);
            }

            uint32_t cur_active = getFirstHit(packet, node->bounds, active, dirIsNeg, t0, t1_max, records, fullRecord);


            if (cur_active < packet.size) {
                if (node->nPrimitives == 0) {
                    int todo_index;
                    if (dirIsNeg[node->axis]) {
                        todo_index = nodeNum + 1;
                        nodeNum = node->secondChildOffset;
                    }
                    else {
                        todo_index = node->secondChildOffset;
                        nodeNum++;
                    }
                    active = cur_active;

                    TraversalNode stack_node(todo_index, cur_active);
                    stack[todoOffset++] = stack_node;
                    assert(todoOffset<64);
                }
                else {
                    // TODO: SIMDize by rewriting hit function
                    // Better to use large packet.
                    uint32_t lastActive = getLastHit(packet, node->bounds, active, dirIsNeg, t0, t1_max, records, fullRecord);
                    for (uint32_t i = active; i < lastActive; i++) {
                        for (uint32_t j = 0; j < node->nPrimitives; j++) {
                            // Use full record, since we still don't know how to deal with shadow ray (yet).
                            primitives[node->primitivesOffset + j]->hit(packet.rays[i], t0, records[i].t, records[i], fullRecord);
                        }
                    }

                    if (todoOffset == 0)
                        break;
                    nodeNum = stack[--todoOffset].node_index;
                    active = stack[todoOffset].active;
                }
            }
            else {
                if (todoOffset == 0)
                    break;
                nodeNum = stack[--todoOffset].node_index;
                active = stack[todoOffset].active;
            }
        }

        // Set t value of rays that miss all prim to negative.
        // So we can go early out function getColor
        for (uint32_t i = 0; i < packet.size; i++) {
            records[i].t = (records[i].t >= t1 - 1e-3) ? -1 :
                records[i].t;
        }
    }

    Geometry* BVHAccel::hit(const Ray& ray, const real_t t0, const real_t t1, hitRecord& h, bool fullRecord) const
    {
        if(!nodes) return NULL;
        Vector3 invDir(1.f / ray.d.x, 1.f / ray.d.y, 1.f / ray.d.z);
        uint32_t dirIsNeg[3] = { invDir.x < 0, invDir.y < 0, invDir.z < 0 };
        // Follow ray through BVH nodes to find primitive intersections
        uint32_t todoOffset = 0, nodeNum = 0;
        uint32_t todo[64];

        real_t minT = t1;
        hitRecord h1;
        Geometry* obj = NULL;
        while (true) {
            const LinearBVHNode *node = &nodes[nodeNum];
            // Check ray against BVH node
            //if (node->bounds.hit(ray, t0, minT)) {
            if (node->bounds.hit(invDir,ray.e, t0, minT,dirIsNeg)) {
                if (node->nPrimitives > 0) {

                    for (uint32_t i = 0; i < node->nPrimitives; ++i)
                    {
                        if (primitives[node->primitivesOffset+i]->hit(ray, t0, minT, h1, fullRecord))
                        {
                            if(minT>h1.t)
                            {
                                obj = primitives[node->primitivesOffset+i];
                                minT = h1.t;
                                h = h1;
                                if(!fullRecord) return obj;
                            }
                        }
                    }
                    if (todoOffset == 0) break;
                    nodeNum = todo[--todoOffset];
                }
                else {
                    // Put far BVH node on _todo_ stack, advance to near node
                    if (dirIsNeg[node->axis]) {
                        todo[todoOffset++] = nodeNum + 1;
                        nodeNum = node->secondChildOffset;
                    }
                    else {
                        todo[todoOffset++] = node->secondChildOffset;
                        nodeNum = nodeNum + 1;
                    }
                }
            }
            else {
                if (todoOffset == 0) break;
                nodeNum = todo[--todoOffset];
            }
        }
        return obj;
    }
}/* _462 */
