#include "PointLinkNode.h"

#include "../../Base/NavMath.h"

bool PointLinkNode::IsPointConvex() const
{
	Vector3 prePoint = preNode->point;
	Vector3 nextPoint = nextNode->point;
	Vector3 preVector = point - prePoint;
	Vector3 nextVector = point - nextPoint;
	return NavMath::Cross(
		Vector3(preVector), Vector3(nextVector)
	).z > 0;
}
