#pragma once
#include "../../Base/Graphes.h"
#include "../../Base/Vectors.h"

using namespace ZXNavMesh;

class PointLinkNode
{
public:
	PointLinkNode(Vector2 point){this->point = point;}

	// 当前点
	Vector2 point;
	// 双向链
	PointLinkNode* preNode = nullptr;
	PointLinkNode* nextNode = nullptr;

	// 获取向前线段
	Line* GetPreLine();
	// 获取向后线段
	Line* getNextLine();
	
	// 计算当前角是否为凸角

private:
	Line* preLine = nullptr;
	Line* nextLine = nullptr;
};
