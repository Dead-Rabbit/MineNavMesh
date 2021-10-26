#include "PointLinkNode.h"

Line* PointLinkNode::GetPreLine()
{
	if (preLine == nullptr)
		preLine = new Line(preNode->point, point);
	return preLine;
}

Line* PointLinkNode::getNextLine()
{
	if (nextLine == nullptr)
		nextLine = new Line(point, nextNode->point);
	return nextLine;
}
