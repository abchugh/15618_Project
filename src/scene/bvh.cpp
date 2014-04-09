#include "bvh.hpp"
#include "scene/scene.hpp"
#include <SDL_timer.h>
#include <stack>
#include <pthread.h>
#include <omp.h>

using namespace std;
namespace _462 {
    const int nBuckets = 8;
    const int numThreads = 2;

    static real_t BoxArea(BoundingBox b)
    {
        real_t v = 1.0;
        for(int i=0;i<3;i++)
        {
            int j = (i+1)%3;
            v+=2*(b.highCoord[i]-b.lowCoord[i])*(b.highCoord[j]-b.lowCoord[j]);
        }
        return v;
    }
    real_t getValue(BoundingBox leftBox, BoundingBox rightBox, const vector<Geometry*> geometries)
    {
        real_t leftArea = BoxArea( leftBox );
        real_t rightArea = BoxArea( leftBox );
        int leftCount = 0, rightCount = 0;

        for(unsigned int i=0;i<geometries.size();i++)
        {
            if(leftBox.hit(geometries[i]->bb))
                leftCount++;
                
            if(rightBox.hit(geometries[i]->bb))
                rightCount++;
        }

        return ( leftArea*leftCount + rightArea*rightCount);
    }
    
    BVHAccel::BVHAccel(const vector<Geometry*>& geometries, uint32_t mp, const string &sm):nodes(NULL) 
    {
        time_t startTime = SDL_GetTicks();

        printf("Building BVH...\n");
        maxPrimsInNode = min(255u, mp);
	primitives = geometries;
        
        if (sm == "middle") splitMethod = SPLIT_MIDDLE;
        else if (sm == "equal")    splitMethod = SPLIT_EQUAL_COUNTS;
        else/*if (sm == "sah")*/                 splitMethod = SPLIT_SAH;
        
        if (primitives.size() == 0)
            return;
        
        // Initialize _buildData_ array for primitives
        vector<BVHPrimitiveInfo> buildData;
        buildData.resize(primitives.size());
        #pragma omp parallel for
        for (uint32_t i = 0; i < primitives.size(); ++i) {
	    buildData[i] = BVHPrimitiveInfo(i, primitives[i]->bb);
        }
        uint32_t totalNodes = 0;
        vector< Geometry* > orderedPrims;
        orderedPrims.reserve(primitives.size());

	/*
        BVHBuildNode *root = recursiveBuild(buildData, 0, primitives.size(), 
					    &totalNodes, orderedPrims);
	*/
	
        BVHBuildNode *root = iterativeBuild(buildData,
					    &totalNodes, orderedPrims);
	
        primitives.swap(orderedPrims);
        
         // Compute representation of depth-first traversal of BVH tree
        nodes = new LinearBVHNode[totalNodes];
        //for (uint32_t i = 0; i < totalNodes; ++i)
            //new (&nodes[i]) LinearBVHNode;
        uint32_t offset = 0;
        flattenBVHTree(root, &offset);
        assert(offset == totalNodes);
        
        time_t endTime = SDL_GetTicks();
        
        printf("Done Building BVH in %d \n", endTime-startTime);
    }

    struct CompareToMid {
        CompareToMid(int d, float m) { dim = d; mid = m; }
        int dim;
        float mid;
        bool operator()(const BVHPrimitiveInfo &a) const {
            return a.centroid[dim] < mid;
        }
    };


    struct ComparePoints {
        ComparePoints(int d) { dim = d; }
        int dim;
        bool operator()(const BVHPrimitiveInfo &a,
                        const BVHPrimitiveInfo &b) const {
            return a.centroid[dim] < b.centroid[dim];
        }
    };


    struct CompareToBucket {
        CompareToBucket(int split, int num, int d, const BoundingBox &b)
            : centroidBounds(b)
        { splitBucket = split; nBuckets = num; dim = d; }
        bool operator()(const BVHPrimitiveInfo &p) const;

        int splitBucket, nBuckets, dim;
        const BoundingBox &centroidBounds;
    };
    
    bool CompareToBucket::operator()(const BVHPrimitiveInfo &p) const {
        int b = nBuckets * ((p.centroid[dim] - centroidBounds.lowCoord[dim]) /
            (centroidBounds.highCoord[dim] - centroidBounds.lowCoord[dim]));
        if (b == nBuckets) b = nBuckets-1;
        assert(b >= 0 && b < nBuckets);
        return b <= splitBucket;
    }

    BVHAccel::~BVHAccel() {
        if(nodes)
        {
            delete []nodes;
            nodes = NULL;
        }
    }

    BVHBuildNode* BVHAccel::SAHSplit(std::vector<BVHPrimitiveInfo> &buildData, uint32_t start, uint32_t end, 
				     BoundingBox bbox, BoundingBox centroidBounds,
				     int dim, 
				     std::vector<Geometry*> &orderedPrims, BVHBuildNode *node, uint32_t& mid) {
	float centroidsLow = centroidBounds.lowCoord[dim];
	float centroidsHigh = centroidBounds.highCoord[dim];
	uint32_t nPrimitives = end - start;

	// Partition primitives using approximate SAH
	if (nPrimitives <= 4) {
	    // Partition primitives into equally-sized subsets
	    mid = (start + end) / 2;
	    std::nth_element(&buildData[start], &buildData[mid],
			     &buildData[end-1]+1, ComparePoints(dim));
	}
	else {
	    // Allocate _BucketInfo_ for SAH partition buckets
	    BucketInfo buckets[nBuckets];
	    
	    BucketInfo sumBuckets[nBuckets * numThreads];
	    float cost[nBuckets-1];
	    time_t startTime = SDL_GetTicks();
            #pragma omp parallel num_threads(numThreads)
	    {
		int threadId = omp_get_thread_num();
		int threadNum = omp_get_num_threads();
		/*
		time_t startTime;
		time_t endTime;
		if (start == 0 && threadId == 0)
		    startTime = SDL_GetTicks();
		*/

		int taskSize = (nPrimitives + threadNum - 1) / threadNum;
		int taskStart = taskSize * threadId + start;
		int taskEnd = taskStart + taskSize;
		BucketInfo localBuckets[nBuckets];

		// Initialize _BucketInfo_ for SAH partition buckets
		// Hash into local buckets in parallel.
		for (int i = taskStart; i < end && i < taskEnd; ++i) {
		    int b = nBuckets *
			((buildData[i].centroid[dim] - centroidsLow) /
			 (centroidsHigh - centroidsLow));
		    if (b >= nBuckets) b = nBuckets-1;
		    if (b < 0) b = 0;
		    assert(b >= 0 && b < nBuckets);
		    localBuckets[b].count++;
		    localBuckets[b].bounds.AddBox(buildData[i].bounds);
		}
		for (int i = 0; i < nBuckets; i++) {
		    sumBuckets[threadId * nBuckets + i].count = localBuckets[i].count;
		    sumBuckets[threadId * nBuckets + i].bounds.AddBox(localBuckets[i].bounds);
		}
		/*
		if (start == 0 && threadId == 0) {
		    endTime = SDL_GetTicks();
		    printf("Thread 0: %d; %d\n", endTime - startTime, nPrimitives);
		}
		*/
	    }
	    time_t endTime = SDL_GetTicks();
	    if (start == 0)
		printf("Local: %d; %d\n", endTime - startTime, nPrimitives);

	    // Merge.
	    startTime = SDL_GetTicks();
	    for (int i = 0; i < numThreads; i++) {
		for (int j = 0; j < nBuckets; j++) {
		    buckets[j].count += sumBuckets[i * nBuckets + j].count;
		    buckets[j].bounds.AddBox(sumBuckets[i * nBuckets + j].bounds);
		}
	    }
	    endTime = SDL_GetTicks();
	    if (start == 0)
		printf("Merge: %d; %d\n", endTime - startTime, nPrimitives);

	    /*
                #pragma omp critical 
		{
		for (int i = 0; i < nBuckets; i++) {
		    buckets[i].count += localBuckets[i].count;
		    buckets[i].bounds.AddBox(localBuckets[i].bounds);
		}
	        }
	    */
		/*
		for (int j = 0; j < threadNum; j++) {
		    for (int i = bTaskStart; i < nBuckets && i < bTaskEnd; i++) {
			buckets[i].count += sumBuckets[i + j * nBuckets].count;
			buckets[i].bounds.AddBox(sumBuckets[i + j * nBuckets].bounds);
		    }
		    }*/
	    startTime = SDL_GetTicks();
	    #pragma omp parallel num_threads(numThreads)
	    {
		int threadId = omp_get_thread_num();
		int threadNum = omp_get_num_threads();
	
		// Changes to view of buckets.
		int bTaskSize = (nBuckets + threadNum - 1) / threadNum;
		int bTaskStart = bTaskSize * threadId;
		int bTaskEnd = bTaskSize * (threadId + 1);

		// Compute costs for splitting after each bucket
		for (int i = bTaskStart; i < (nBuckets-1) && i < bTaskEnd; ++i) {
		    BoundingBox b0, b1;
		    int count0 = 0, count1 = 0;
		    for (int j = 0; j <= i; ++j) {
			b0.AddBox(buckets[j].bounds);
			count0 += buckets[j].count;
		    }
		    for (int j = i+1; j < nBuckets; ++j) {
			b1.AddBox(buckets[j].bounds);
			count1 += buckets[j].count;
		    }
		    cost[i] = .125f + (count0*b0.SurfaceArea() + count1*b1.SurfaceArea()) /
			bbox.SurfaceArea();
		}
	    }
	    endTime = SDL_GetTicks();
	    if (start == 0)
		printf("Cost: %d; %d\n\n", endTime - startTime, nPrimitives);

	    // Find bucket to split at that minimizes SAH metric
	    float minCost = cost[0];
	    uint32_t minCostSplit = 0;
	    for (int i = 1; i < nBuckets-1; ++i) {
		if (cost[i] < minCost) {
		    minCost = cost[i];
		    minCostSplit = i;
		}
	    }

	    // Either create leaf or split primitives at selected SAH bucket
	    if (nPrimitives > maxPrimsInNode ||
		minCost < nPrimitives) {
		BVHPrimitiveInfo *pmid = std::partition(&buildData[start],
							&buildData[end-1]+1,
							CompareToBucket(minCostSplit, nBuckets, dim, centroidBounds));
		mid = pmid - &buildData[0];
	    }
	    else {
		// Create leaf _BVHBuildNode_
		uint32_t firstPrimOffset = orderedPrims.size();
		for (uint32_t i = start; i < end; ++i) {
		    uint32_t primNum = buildData[i].primitiveNumber;
		    // TODO: Would be affected by parallelization.
		    orderedPrims.push_back(primitives[primNum]);
		}
		node->InitLeaf(firstPrimOffset, nPrimitives, bbox);
	    }
	}

	return node;
    }

    int multithreadedSAHSplit(std::vector<BVHPrimitiveInfo> &buildData, uint32_t start, uint32_t end,
					BoundingBox bbox, BoundingBox centroidBounds, float* cost,
			      int dim, std::vector<Geometry*> primitives,
					std::vector<Geometry*> &orderedPrims, BVHBuildNode *node,
					BuildArg* buildArg, pthread_barrier_t &barrier) {
	int threadId = buildArg->threadId;
	int threadNum = buildArg->threadNum;
	BucketInfo *buckets = buildArg->buckets;

	float centroidsLow = centroidBounds.lowCoord[dim];
	float centroidsHigh = centroidBounds.highCoord[dim];
	uint32_t nPrimitives = end - start;
	int split = (start + end) / 2;

	// Partition primitives using approximate SAH
	if (nPrimitives <= 4) {
	    split = (start + end) / 2;
	    if (threadId == 0) {
		// Partition primitives into equally-sized subsets
		std::nth_element(&buildData[start], &buildData[split],
				 &buildData[end-1]+1, ComparePoints(dim));
	    }
	}
	else {
	    // Allocate _BucketInfo_ for SAH partition buckets
	    int taskSize = (nPrimitives + threadNum - 1) / threadNum;
	    int taskStart = taskSize * threadId + start;
	    int taskEnd = taskStart + taskSize;
	    BucketInfo localBuckets[nBuckets];

	    // Initialize _BucketInfo_ for SAH partition buckets
	    // Hash into local buckets in parallel.
	    for (int i = taskStart; i < end && i < taskEnd; ++i) {
		int b = nBuckets *
		    ((buildData[i].centroid[dim] - centroidsLow) /
		     (centroidsHigh - centroidsLow));
		if (b == nBuckets) b = nBuckets-1;
		assert(b >= 0 && b < nBuckets);
		localBuckets[b].count++;
		localBuckets[b].bounds.AddBox(buildData[i].bounds);
	    }
	    for (int i = 0; i < nBuckets; i++) {
		buckets[threadId * nBuckets + i].count = localBuckets[i].count;
		buckets[threadId * nBuckets + i].bounds.AddBox(localBuckets[i].bounds);
	    }
	    pthread_barrier_wait(&barrier);

	    // Merge.
	    if (threadId == 0) {
		for (int i = 1; i < threadNum; i++) {
		    for (int j = 0; j < nBuckets; j++) {
			buckets[j].count += buckets[i * nBuckets + j].count;
			buckets[j].bounds.AddBox(buckets[i * nBuckets + j].bounds);
		    }
		}
	    }
	    pthread_barrier_wait(&barrier);

	    // Changes to view of buckets.
	    int bTaskSize = (nBuckets + threadNum - 1) / threadNum;
	    int bTaskStart = bTaskSize * threadId;
	    int bTaskEnd = bTaskSize * (threadId + 1);

	    // Compute costs for splitting after each bucket
	    for (int i = bTaskStart; i < (nBuckets-1) && i < bTaskEnd; ++i) {
		BoundingBox b0, b1;
		int count0 = 0, count1 = 0;
		for (int j = 0; j <= i; ++j) {
		    b0.AddBox(buckets[j].bounds);
		    count0 += buckets[j].count;
		}
		for (int j = i+1; j < nBuckets; ++j) {
		    b1.AddBox(buckets[j].bounds);
		    count1 += buckets[j].count;
		}
		cost[i] = .125f + (count0*b0.SurfaceArea() + count1*b1.SurfaceArea()) /
		    bbox.SurfaceArea();
	    }
	    pthread_barrier_wait(&barrier);

	    // Find bucket to split at that minimizes SAH metric
	    float minCost = cost[0];
	    uint32_t minCostSplit = 0;
	    for (int i = 1; i < nBuckets-1; ++i) {
		if (cost[i] < minCost) {
		    minCost = cost[i];
		    minCostSplit = i;
		}
	    }

	    if (threadId == 0) {
		// Either create leaf or split primitives at selected SAH bucket
		// TODO: minCost?
		if (nPrimitives > buildArg->maxPrimsInNode ||
		    minCost < nPrimitives) {
		    BVHPrimitiveInfo *pmid = std::partition(&buildData[start],
							    &buildData[end-1]+1,
							    CompareToBucket(minCostSplit, nBuckets, dim, centroidBounds));
		    split = pmid - &buildData[0];
		}
		else {
		    // Create leaf _BVHBuildNode_
		    uint32_t firstPrimOffset = orderedPrims.size();
		    for (uint32_t i = start; i < end; ++i) {
			uint32_t primNum = buildData[i].primitiveNumber;
			// TODO: Would be affected by parallelization.
			orderedPrims.push_back(primitives[primNum]);
		    }
		    node->InitLeaf(firstPrimOffset, nPrimitives, bbox);
		    split = -1;
		}
	    }
	}

	pthread_barrier_wait(&barrier);

	return split;
    }

    void* multithreadedBuild(void* arg) {	
	BuildArg* buildArg = (BuildArg*)arg;
	std::stack<std::pair<int32_t, int32_t>>* infoStackPtr = buildArg->infoStackPtr;
	std::vector<BVHPrimitiveInfo>* buildDataPtr = buildArg->buildDataPtr;
	int threadId = buildArg->threadId;
	int threadNum = buildArg->threadNum;
	BVHBuildNode* parent = NULL;
	std::stack<BVHBuildNode*> parentPtrStack;
	//	parentPtrStack.push(parent);

	BVHBuildNode* node = NULL;
	BVHBuildNode* root = NULL;

	BoundingBox* bboxes = buildArg->bboxes;
	BoundingBox* centroidBounds = buildArg->centroidBounds;

	time_t startTime;
	time_t endTime;

	while (!infoStackPtr->empty()) {
	    std::pair<int32_t, int32_t> range = infoStackPtr->top();
	    int start = range.first;
	    int end = range.second;
	    //printf("Thread %d: grasp job %d - %d.\n", threadId, start, end);
	    bool left = (start < 0);
	    start = (start < 0) ? -1 * start - 1 : start;
	    int split = (start + end) / 2;

	    //printf("Thread %d: grasp job %d - %d.\n", threadId, start, end);
	    assert(start != end);
	    pthread_barrier_wait(buildArg->barrierPtr);
	    
	    if (threadId == 0) {
		startTime = SDL_GetTicks();
		infoStackPtr->pop();
		node = new BVHBuildNode();
		if (root == NULL)
		    root = node;

		if (parentPtrStack.size() > 0) {
		    parent = parentPtrStack.top();

		    if (!left)
			parentPtrStack.pop();
		    if (parent != NULL) {
			parent->InitChild(node, left);
		    }
		}
		(*(buildArg->totalNodes))++;
	    }

	    // TODO: Barrier?
	    // Compute the large bounding box for the node in parallel.
	    BoundingBox localNodeBbox;
	    int nPrimitives = end - start;
	    int localSize = (nPrimitives + buildArg->threadNum - 1) / buildArg->threadNum;
	    for (int i = start + threadId * localSize; i < end && 
		     i < start + (threadId + 1) * localSize; i++) {
		localNodeBbox.AddBox(buildDataPtr->at(i).bounds);
	    }
	    bboxes[threadId] = localNodeBbox;

	    pthread_barrier_wait(buildArg->barrierPtr);

	    // Merge the large bounding box at bboxes[0].
	    if (threadId == 0) {
		endTime = SDL_GetTicks();
		if (start == 0)
		printf("Build local large bb: %d\n", endTime - startTime);
		startTime = SDL_GetTicks();
		for (int i = 1; i < threadNum; i++) {
		    bboxes[0].AddBox(bboxes[i]);
		}
		endTime = SDL_GetTicks();
		if (start == 0)
		printf("Merge large bb: %d\n", endTime - startTime);

	    }
	    //	    printf("after large merge.\n");
	    pthread_barrier_wait(buildArg->barrierPtr);
	    
	    if (nPrimitives == 1) {
		if (threadId == 0) {
		    int firstPrimOffset = buildArg->orderedPrimsPtr->size();
		    for (int i = start; i < end; ++i) {
			int primNum = buildDataPtr->at(i).primitiveNumber;
			buildArg->orderedPrimsPtr->push_back(buildArg->primitivesPtr->at(primNum));
		    }
		    node->InitLeaf(firstPrimOffset, nPrimitives, bboxes[0]);
		}
	    }
	    else {
		// Find largest diverge dimension in parallel.
		startTime = SDL_GetTicks();
		BoundingBox localCentroidBound;
		for (int i = start + threadId * localSize; i < end && 
			 i < start + (threadId + 1) * localSize; i++) {
		    localCentroidBound.AddPoint(buildDataPtr->at(i).centroid);
		}
		centroidBounds[threadId] = localCentroidBound;
		pthread_barrier_wait(buildArg->barrierPtr);

		//		printf("after centroid.\n");
		if (threadId == 0) {
		    for (int i = 1; i < threadNum; i++) {
			centroidBounds[0].AddBox(centroidBounds[i]);
		    }
		    *(buildArg->dimPtr) = centroidBounds[0].MaximumExtent();
		    endTime = SDL_GetTicks();
		if (start == 0)
		    printf("Find dim: %d\n", endTime - startTime);

		}
		pthread_barrier_wait(buildArg->barrierPtr);

		int dim = *(buildArg->dimPtr);
		// TODO: Special case here.
		startTime = SDL_GetTicks();
		split = multithreadedSAHSplit((*buildDataPtr), start, end,
					      bboxes[0], centroidBounds[0], buildArg->cost,
					      dim, *(buildArg->primitivesPtr),
					      *(buildArg->orderedPrimsPtr), node,
					      buildArg, *(buildArg->barrierPtr));

		//	    printf("after sah.\n");
		//printf("split: %d\n", split);
		pthread_barrier_wait(buildArg->barrierPtr);
		if (threadId == 0) {
		    endTime = SDL_GetTicks();
		if (start == 0)
		    printf("SAH: %d\n", endTime - startTime);

		    if (split > 0) {
			parentPtrStack.push(node);
			std::pair<int32_t, int32_t> leftChild = std::make_pair(-1 * (start + 1), split);
			std::pair<int32_t, int32_t> rightChild = std::make_pair(split, end);
			infoStackPtr->push(rightChild);
			infoStackPtr->push(leftChild);
		    }
		}
	    }
	    pthread_barrier_wait(buildArg->barrierPtr);
	}

	return root;
    }

    BVHBuildNode* BVHAccel::iterativeBuild(std::vector<BVHPrimitiveInfo> &buildData, 
					   uint32_t *totalNodes, std::vector<Geometry*> &orderedPrims) {
	std::pair<int32_t, int32_t> initInfo(-1, primitives.size());
	std::stack<std::pair<int32_t, int32_t>> infoStack;
	infoStack.push(initInfo);
	pthread_barrier_t barrier;

	pthread_barrier_init(&barrier, NULL, numThreads);

	pthread_t threads[numThreads];
	BuildArg buildArgs[numThreads];
	BucketInfo buckets[nBuckets * numThreads];
	float cost[nBuckets];
	BoundingBox bboxes[numThreads];
	BoundingBox centroidBounds[numThreads];
	int dim = 0;
	    printf("###%ld - %d\n", &buildData, buildData.size());	
	for (int i = 0; i < numThreads; i++) {
	    buildArgs[i].buildDataPtr = &buildData;

	    buildArgs[i].totalNodes = totalNodes;
	    buildArgs[i].orderedPrimsPtr = &orderedPrims;
	    buildArgs[i].primitivesPtr = &primitives;
	    buildArgs[i].maxPrimsInNode = maxPrimsInNode;
	    buildArgs[i].threadId = i;
	    buildArgs[i].threadNum = numThreads;
	    buildArgs[i].barrierPtr = &barrier;
	    buildArgs[i].infoStackPtr = &infoStack;
	    buildArgs[i].buckets = buckets;
	    buildArgs[i].cost = cost;
	    buildArgs[i].dimPtr = &dim;
	    buildArgs[i].bboxes = bboxes;
	    buildArgs[i].centroidBounds = centroidBounds;

	    if (i > 0) {
		int rc = pthread_create(&threads[i], NULL, multithreadedBuild, &buildArgs[i]);
		assert(rc == 0);
	    }
	}

	BVHBuildNode* node = (BVHBuildNode*)multithreadedBuild(&buildArgs[0]);
	for (int i = 1; i < numThreads; i++) {
	    pthread_join(threads[i], NULL);
	}

	return node;
    }


    BVHBuildNode *BVHAccel::recursiveBuild( vector<BVHPrimitiveInfo> &buildData, uint32_t start,
					    uint32_t end, uint32_t *totalNodes, vector<Geometry* > &orderedPrims){
	assert(start != end);
	(*totalNodes)++;
	BVHBuildNode *node = new BVHBuildNode();
	// Compute bounds of all primitives in BVH node
	BoundingBox bbox;

	for (uint32_t i = start; i < end; ++i)
	    bbox.AddBox(buildData[i].bounds);

	uint32_t nPrimitives = end - start;
	if (nPrimitives == 1) {
	    // Create leaf _BVHBuildNode_
	    uint32_t firstPrimOffset = orderedPrims.size();
	    for (uint32_t i = start; i < end; ++i) {
		uint32_t primNum = buildData[i].primitiveNumber;
		orderedPrims.push_back(primitives[primNum]);
	    }
	    node->InitLeaf(firstPrimOffset, nPrimitives, bbox);
	}
	else {
	    // Compute bound of primitive centroids, choose split dimension _dim_
	    // TODO: Find min and max in parallel? Do the same to all the three dimensions?
	    BoundingBox centroidBounds;

	    for (uint32_t i = start; i < end; ++i)
		centroidBounds.AddPoint(buildData[i].centroid);
	    int dim = centroidBounds.MaximumExtent();

	    // Partition primitives into two sets and build children
	    // TODO: Special case. Handle later.
	    uint32_t mid = (start + end) / 2;
	    if (centroidBounds.highCoord[dim] == centroidBounds.lowCoord[dim]) {
		// Create leaf _BVHBuildNode_
		uint32_t firstPrimOffset = orderedPrims.size();
		for (uint32_t i = start; i < end; ++i) {
		    uint32_t primNum = buildData[i].primitiveNumber;
		    orderedPrims.push_back(primitives[primNum]);
		}
		node->InitLeaf(firstPrimOffset, nPrimitives, bbox);
		return node;
	    }

	    // Partition primitives based on _splitMethod_
	    switch (splitMethod) {
	    case SPLIT_MIDDLE: {
		// Partition primitives through node's midpoint
		double pmid = .5f * (centroidBounds.lowCoord[dim] + centroidBounds.highCoord[dim]);
		BVHPrimitiveInfo *midPtr = std::partition(&buildData[start],
							  &buildData[end-1]+1,
							  CompareToMid(dim, pmid));
		mid = midPtr - &buildData[0];
		if (mid != start && mid != end)
		    // for lots of prims with large overlapping bounding boxes, this
		    // may fail to partition; in that case don't break and fall through
		    // to SPLIT_EQUAL_COUNTS
		    break;
	    }
	    case SPLIT_EQUAL_COUNTS: {
		// Partition primitives into equally-sized subsets
		mid = (start + end) / 2;
		std::nth_element(&buildData[start], &buildData[mid],
				 &buildData[end-1]+1, ComparePoints(dim));
		break;
	    }
	    case SPLIT_SAH: default: {
		SAHSplit(buildData, start, end, bbox, centroidBounds,
			 dim, orderedPrims, node, mid);
		break;
	    }
	    }
	    node->InitInterior(dim,
			       recursiveBuild(buildData, start, mid,
					      totalNodes, orderedPrims),
			       recursiveBuild(buildData, mid, end,
					      totalNodes, orderedPrims));
	}
	return node;
    }
    uint32_t BVHAccel::flattenBVHTree(BVHBuildNode *node, uint32_t *offset)
    {
	LinearBVHNode *linearNode = &nodes[*offset];
	linearNode->bounds = node->bounds;
	uint32_t myOffset = (*offset)++;
	if (node->nPrimitives > 0) {
	    //	    printf("%ld: l: %ld; r: %ld\n", node, node->children[0], node->children[1]);
	    assert(!node->children[0]);
	    assert(!node->children[1]);
	    linearNode->primitivesOffset = node->firstPrimOffset;
	    linearNode->nPrimitives = node->nPrimitives;
	}
	else {
	    // Creater interior flattened BVH node
	    linearNode->axis = node->splitAxis;
	    linearNode->nPrimitives = 0;
	    flattenBVHTree(node->children[0], offset);
	    linearNode->secondChildOffset = flattenBVHTree(node->children[1],
							   offset);
	}
	return myOffset;
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
