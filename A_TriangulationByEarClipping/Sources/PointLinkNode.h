#pragma once
#include "../../Base/Graphes.h"
#include "../../Base/Vectors.h"

using namespace ZXNavMesh;

class PointLinkNode
{
public:
	PointLinkNode(int num, Vector3 point)
	{
		this->num = num;
		this->point = point;
	}

	int num;
	
	// 当前点
	Vector3 point;
	// 双向链
	PointLinkNode* preNode = nullptr;
	PointLinkNode* nextNode = nullptr;

	// 计算当前角是否为凸角
	bool IsPointConvex() const;
private:
	Line* preLine = nullptr;
	Line* nextLine = nullptr;
};
