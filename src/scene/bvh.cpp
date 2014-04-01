#include "bvh.hpp"
#include "scene/scene.hpp"
namespace _462 {
	
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
	real_t getValue(BoundingBox leftBox, BoundingBox rightBox, const std::vector<Geometry*> geometries)
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
	void bvhNode::GetChildNodeBounds(const std::vector<Geometry*> geometries, BoundingBox& box1, BoundingBox& box2)const
	{
		real_t minCost = 1e30;
		
		for(int b=0;b<3;b++)
		{
			//Checking all possibilities takes a lot of time to initialise so leaving it out	
			/*
			for(int i=0;i<geometries.size();i++){
				real_t testValues[] = {geometries[i]->bb.highCoord[b], geometries[i]->bb.lowCoord[b]};
				for(int j=0;j<2;j++)
				{*/
			real_t testValue = (bb.lowCoord[b]+bb.highCoord[b])/2;//testValues[j];//
			BoundingBox leftBox = bb, rightBox = bb;	
			leftBox.highCoord[b] = rightBox.lowCoord[b] = testValue;
			real_t cost = getValue(leftBox,rightBox,geometries);
			if(cost<minCost)
			{
				minCost = cost;
				box1 = leftBox;
				box2 = rightBox;
			}
				/*}}*/
		}
	}
	bvhNode::bvhNode(const std::vector<Geometry*> geometries):left(NULL),right(NULL)
	{
		size = geometries.size();
		for(unsigned int i=0;i<geometries.size();i++)
			bb.AddBox(geometries[i]->bb);
		if(geometries.size()<3)
			entries = geometries;
		else
		{
			BoundingBox leftBox = bb, rightBox = bb;
			GetChildNodeBounds(geometries,leftBox,rightBox);
			
			std::vector<Geometry*> leftGeometries, rightGeometries;
			for(unsigned int i=0;i<geometries.size();i++)
			{
				if(leftBox.hit(geometries[i]->bb))
					leftGeometries.push_back(geometries[i]);
				
				if(rightBox.hit(geometries[i]->bb))
					rightGeometries.push_back(geometries[i]);
			}

			if(leftGeometries.size()>0 && rightGeometries.size()>0 && leftGeometries.size()<geometries.size() && rightGeometries.size()<geometries.size() )
			{
				left = new bvhNode(leftGeometries);
				right = new bvhNode(rightGeometries);
			}
			else
				entries = geometries;
		}
	}

	bvhNode::~bvhNode()
	{
		if(left)
		{
			delete left;
			left = NULL;
		}
		if(right)
		{
			delete right;
			right = NULL;
		}
	}

	Geometry* bvhNode::hit(const Ray& r, const real_t t0, const real_t t1, hitRecord& h, bool fullRecord) const
	{
		if(!bb.hit(r,t0,t1))
			return NULL;
		
		Geometry* obj = NULL;
		real_t minT = t1;
		hitRecord h1;

		if(left==NULL || right==NULL)
		{
			minT = t1*2;
			for(unsigned int i=0;i<entries.size();i++)
				if(entries[i]->hit(r,t0,minT,h1,fullRecord))
				{
					if(minT>h1.t)
					{
						obj = entries[i];
						minT = h1.t;
						h = h1;
						if(!fullRecord) return entries[i];
					}
				}

			if(minT>t1)
				return NULL;
			else
				return obj;
		}

		Geometry* obj1 = NULL;

		if(left!=NULL && (obj1=left->hit(r,t0,minT,h1,fullRecord)) && minT>h1.t)
		{
			minT = h1.t;
			h = h1;
			obj = obj1;
			if(!fullRecord) return obj;
		}
		
		if(right!=NULL && (obj1=right->hit(r,t0,minT,h1,fullRecord)) && minT>h1.t)
		{
			minT = h1.t;
			h = h1;
			obj = obj1;
			if(!fullRecord) return obj;
		}
		if(minT<t1)
			return obj;
		else
			return NULL;
	}

}/* _462 */