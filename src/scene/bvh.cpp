#include "bvh.hpp"
#include "scene/scene.hpp"
#include <SDL_timer.h>
#include <queue>
#include <omp.h>

using namespace std;
namespace _462 {
	
time_t t1[20], t2[20], t3[20], t4[20], t5[20], t6[20], t7[20], t8[20], tP[20];	
//#define ENABLED_TIME_LOGS
#ifdef ENABLED_TIME_LOGS
	
	time_t _prevTick[20];

#define AddTimeSincePreviousTick(interval)\
	{int tid = omp_get_thread_num();\
	time_t timeNew = SDL_GetTicks();\
	interval[tid] += timeNew - _prevTick[tid];\
	_prevTick[tid] = timeNew;}

#define GetTime(timeNew) time_t timeNew = SDL_GetTicks();\
	_prevTick[omp_get_thread_num()] = timeNew;

#else
	#define AddTimeSincePreviousTick(interval)
	#define GetTime(timeNew)
#endif
	static time_t sum(time_t arr[])
	{
		time_t sum = 0;
		for(int i=0;i<20;i++)
			sum+=arr[i];
		return sum;
	}
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
		else if (sm == "equal")  splitMethod = SPLIT_EQUAL_COUNTS;
		else/*if (sm == "sah")*/         splitMethod = SPLIT_SAH;
		
		if (primitives.size() == 0)
			return;
		
		// Initialize _buildData_ array for primitives
		vector<BVHPrimitiveInfo> buildData;
		buildData.reserve(primitives.size());
		for (uint32_t i = 0; i < primitives.size(); ++i) {
			buildData.push_back(BVHPrimitiveInfo(i, primitives[i]->bb));
		}
		uint32_t totalNodes = 0;
		vector< Geometry* > orderedPrims(primitives.size());
		
		queueData rootData = {0, static_cast<uint32_t>(primitives.size()),NULL, true, NULL };
		rootData.isValid = new bool;
		*rootData.isValid = true;
		q[0].push_back(rootData);

		int working = 0;
		

		time_t endTime = SDL_GetTicks();
		printf("Start phase  in %ld \n", endTime-startTime);

		BVHBuildNode* root = NULL;
		time_t busy[20] = {0}, idle[20] = {0}, idleX[20] = {0};
		
		printf("omp_maxThreads: %d\n\n",omp_get_max_threads());
		int blah = 0;
#pragma omp parallel 
		{
			
			int tid = omp_get_thread_num();
			int backoff = 2;
			while(1)
			{
				blah++;

				time_t idleStart = SDL_GetTicks();
				queueData data;
				bool foundWork = false;
				
				#pragma omp critical(queueUpdate)
				{
					while(!q[tid].empty())
					{
						data = q[tid].back();
						q[tid].pop_back();
						if(!*data.isValid) {
							delete data.isValid;
							data.isValid = NULL;
							continue;
						}
						*data.isValid = false;
						working |= (1<<tid);
						foundWork = true;
						break;
					}
					if(!foundWork) while(!pq.empty())
					{	
						data = pq.top();	
						pq.pop();
						if(*data.isValid==false) {
							delete data.isValid;
							data.isValid = NULL;
							continue;
						}
						*data.isValid = false;

						working |= (1<<tid);
						foundWork = true;
						break;
					}
					if(!foundWork)
						working &= 0xFFFF - (1<<tid);
				}
				if(!foundWork)
				{	
						SDL_Delay(backoff);
						if(backoff<4)backoff*=2;
				}
				
				if(blah%10==0 && blah<200)/*if(working& (1<<tid))*/
					printf("%d %lu %ld\n", working,pq.size(),SDL_GetTicks()-endTime );//data.start, data.end);//
				
				if(working == 0) //no one is working, time to end
					break;

				time_t idleEnd = SDL_GetTicks();
					
				idleX[tid] += idleEnd-idleStart;
				if(!foundWork)
					idle[tid] += idleEnd - idleStart;
				
				if(foundWork)
				{
					time_t startT = SDL_GetTicks();
					BVHBuildNode* node = recursiveBuild(buildData, data.start, data.end, &totalNodes, orderedPrims, 
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
		
		printf("End phase in %ld \n", endTime-startTime);
		
		 // Compute representation of depth-first traversal of BVH tree
		nodes = new LinearBVHNode[totalNodes];

		uint32_t offset = 0;
		flattenBVHTree(root, &offset);
		assert(offset == totalNodes);
		
		 endTime = SDL_GetTicks();
		
		/*printf("Phases %lld %lld %lld %lld %lld %lld %lld %lld %lld\n", sum(t1), sum(t2), sum(t3), sum(t4), sum(t5), sum(t6), sum(t7), sum(t8), sum(tP));
		for(int i=0;i<20;i++) if(busy[i]!=0)
			printf("%ld ", busy[i]);
		printf("\n");
		for(int i=0;i<20;i++) if(busy[i]!=0)
			printf("%lld/%lld ", idle[i], idleX[i]);*/
		printf("\n");
		printf("Done Building BVH in %ld \n", endTime-startTime);
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
	//TODO:convert into #define to check for perf improvement
	void BVHAccel::buildLeaf(vector<BVHPrimitiveInfo> &buildData, uint32_t start,
        uint32_t end, vector<Geometry* > &orderedPrims, BVHBuildNode *node, const BoundingBox& bbox)
	{
		GetTime(primitiveStart);
		for (uint32_t i = start; i < end; ++i)
				orderedPrims[i] = primitives[buildData[i].primitiveNumber];
		node->InitLeaf(start, end-start, bbox);
		
		AddTimeSincePreviousTick(tP);
	}
	BVHBuildNode *BVHAccel::recursiveBuild( vector<BVHPrimitiveInfo> &buildData, uint32_t start,
        uint32_t end, uint32_t *totalNodes, vector<Geometry* > &orderedPrims, BVHBuildNode *parent, bool firstChild){
			
		//printf("%d %d %d\n", start,end, omp_get_thread_num());

		assert(start != end);
		GetTime(startTime);

#pragma omp atomic
			(*totalNodes)++;
		
		BVHBuildNode *node = new BVHBuildNode(parent, firstChild);
		if(parent)
			parent->children[firstChild?0:1] = node;

		int numInternalBranches = 0;
		queueData child1Data, child2Data;

		uint32_t nPrimitives = end - start;

		// Compute bounds of all primitives in BVH node
		BoundingBox bbox;
		for (uint32_t i = start; i < end; ++i)
			bbox.AddBox(buildData[i].bounds);
		AddTimeSincePreviousTick(t1);

		if (nPrimitives == 1) {
			buildLeaf(buildData,start,end, orderedPrims,node,bbox);
		}
		else {
			// Compute bound of primitive centroids, choose split dimension _dim_
			BoundingBox centroidBounds;
			for (uint32_t i = start; i < end; ++i)
				centroidBounds.AddPoint(buildData[i].centroid);
			int dim = centroidBounds.MaximumExtent();
			AddTimeSincePreviousTick(t2);


			// Partition primitives into two sets and build children
			uint32_t mid = (start + end) / 2;
			if (centroidBounds.highCoord[dim] == centroidBounds.lowCoord[dim]) {
				buildLeaf(buildData,start,end, orderedPrims,node,bbox);
				goto finishUp;
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
				// Partition primitives using approximate SAH
				if (nPrimitives <= 4) {
					// Partition primitives into equally-sized subsets
					mid = (start + end) / 2;
					std::nth_element(&buildData[start], &buildData[mid],
									 &buildData[end-1]+1, ComparePoints(dim));
					
				
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
						int b = nBuckets *
							((buildData[i].centroid[dim] - centroidBounds.lowCoord[dim]) /
							 (centroidBounds.highCoord[dim] - centroidBounds.lowCoord[dim]));
						if (b == nBuckets) b = nBuckets-1;
						assert(b >= 0 && b < nBuckets);
						buckets[b].count++;
						buckets[b].bounds.AddBox(buildData[i].bounds);
					}
					AddTimeSincePreviousTick(t4);
	
					// Compute costs for splitting after each bucket
					float cost[nBuckets-1];
					for (int i = 0; i < nBuckets-1; ++i) {
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
					AddTimeSincePreviousTick(t5);

					// Find bucket to split at that minimizes SAH metric
					float minCost = cost[0];
					uint32_t minCostSplit = 0;
					for (int i = 1; i < nBuckets-1; ++i) {

						if (cost[i] < minCost) {
							minCost = cost[i];
							minCostSplit = i;
						}
					}
					AddTimeSincePreviousTick(t6);

					// Either create leaf or split primitives at selected SAH bucket
					if (nPrimitives > maxPrimsInNode ||
						minCost < nPrimitives) {
						BVHPrimitiveInfo *pmid = std::partition(&buildData[start],
							&buildData[end-1]+1,
							CompareToBucket(minCostSplit, nBuckets, dim, centroidBounds));
						mid = pmid - &buildData[0];
						
						AddTimeSincePreviousTick(t7);
					}
                
					else {
						buildLeaf(buildData,start,end, orderedPrims,node,bbox);
						goto finishUp;
					}
					
					
				}
				break;
			}
			}
			node->splitAxis = dim;

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
				child2Data.isValid = new bool;
				*child2Data.isValid = true;
				q[omp_get_thread_num()].push_back(child2Data);

				queueData top;
				if(!pq.empty())top=pq.top();
				if(pq.size()<omp_get_max_threads() || (top.end-top.start)/8<child2Data.end-child2Data.start)
				{
					#pragma omp critical(queueUpdate)
						pq.push(child2Data);
				}
			}
		}
		finishUp:
		if(parent)
		{
			assert(!parent->childComplete[firstChild?0:1]);
			parent->childComplete[firstChild?0:1] = true;

			while(parent!=NULL)
			{
				/*not thread safe??*/
				if(parent->childComplete[0] == true && parent->childComplete[1] == true)
					parent->InitInterior(parent->children[0], parent->children[1]);
				else
					break;
				parent = parent->parent;
			}
		}
		
		AddTimeSincePreviousTick(t8);
		if(numInternalBranches>=1)
		{
			//printf("tid: %d continuing %d %d\n", omp_get_thread_num(), data1.start, data1.end);
			recursiveBuild(buildData, child1Data.start, child1Data.end,
					totalNodes, orderedPrims, child1Data.parent, child1Data.isFirstChild);
			if(numInternalBranches==2)
			{
				recursiveBuild(buildData, child2Data.start, child2Data.end,
					totalNodes, orderedPrims, child2Data.parent, child2Data.isFirstChild);
			}
		}
	
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
