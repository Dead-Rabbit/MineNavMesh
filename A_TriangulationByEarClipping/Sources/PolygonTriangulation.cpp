#include "PolygonTriangulation.h"

int PointLinkNode::pointNum = 0;

void PolygonTriangulation::SetPolygonOutPoints(vector<Vector3> edgePoints)
{
    this->edgePoints = edgePoints;
    // 生成链表
    const int pointSize = edgePoints.size();
    PointLinkNode* curNode = firstNode;
    PointLinkNode* preNode = firstNode;
    for (int i = 0; i < pointSize; i++)
    {
        auto point = edgePoints[i];

        // 插入链表节点
        if (firstNode == nullptr)
        {
            firstNode = new PointLinkNode(point.x, point.y, point.z);
            curNode = firstNode;
            continue;
        }
        
        curNode->nextNode = new PointLinkNode(point.x, point.y, point.z);
        preNode = curNode;
        curNode = curNode->nextNode;
        curNode->preNode = preNode;
    }
    curNode->nextNode = firstNode;
    firstNode->preNode = curNode;
    
    // 寻找最靠右的 Edge 点
    curNode = firstNode;
    do
    {
        if (rightEdgeNode == nullptr)
            rightEdgeNode = curNode;
        else if (curNode->point.x > rightEdgeNode->point.x)
            rightEdgeNode = curNode;
        
        curNode = curNode->nextNode;
    }
    while (curNode != firstNode);
}

void PolygonTriangulation::SetPolygonInsidePoints(vector<Vector3> innerPoints)
{
    // 生成新的链表
    const int pointSize = innerPoints.size();
    int pointNumOffset = this->edgePoints.size();
    PointLinkNode* firstInsideNode = nullptr;
    PointLinkNode* curInsideNode = firstInsideNode;
    PointLinkNode* preInsideNode = firstInsideNode;
    for (int i = 0; i < pointSize; i++)
    {
        auto point = innerPoints[i];

        // 插入链表节点
        if (firstInsideNode == nullptr)
        {
            firstInsideNode = new PointLinkNode(point.x, point.y, point.z);
            curInsideNode = firstInsideNode;
            continue;
        }
        
        curInsideNode->nextNode = new PointLinkNode(point.x, point.y, point.z);
        preInsideNode = curInsideNode;
        curInsideNode = curInsideNode->nextNode;
        curInsideNode->preNode = preInsideNode;
    }
    curInsideNode->nextNode = firstInsideNode;
    firstInsideNode->preNode = curInsideNode;

    // 寻找最靠右的 Inside 点
    curInsideNode = firstInsideNode;
    PointLinkNode* rightInsideNode = nullptr;
    do
    {
        if (rightInsideNode == nullptr)
            rightInsideNode = curInsideNode;
        else if (curInsideNode->point.x > rightInsideNode->point.x)
            rightInsideNode = curInsideNode;
        
        curInsideNode = curInsideNode->nextNode;
    }
    while (curInsideNode != firstInsideNode);
    
    // 获取从inner右点触发向x轴正方向走的最大线段
    Line compareLine = Line(rightInsideNode->point, Vector3(rightEdgeNode->point.x + 10, rightInsideNode->point.y, 0));
    // 判断靠右内部节点与哪个线段的相交点最靠近y轴
    PointLinkNode* curNode = firstNode;
    PointLinkNode* suitNode = nullptr;
    Vector3 suitCrossPoint;
    if (curNode == nullptr)
        return ;
    do
    {
        Line curLine = Line(curNode->point, curNode->nextNode->point);
        Vector3 crossPoint;
        if (NavMath::GetSegmentLinesIntersection(compareLine.start, compareLine.end,
            curLine.start, curLine.end, crossPoint))
        {
            if (suitNode == nullptr)
            {
                suitNode = curNode;
                suitCrossPoint = crossPoint;
            } else
            {
                if (crossPoint.x < suitCrossPoint.x)
                {
                    suitNode = curNode;
                    suitCrossPoint = crossPoint;
                }
            }
        }
        
        curNode = curNode->nextNode;
    }
    while (curNode != firstNode);
    
    // 找到最近的线段，检查 形成的三角形内是否有其他点
    curNode = firstNode;
    Vector3 A = rightInsideNode->point, B = suitCrossPoint, C = suitNode->point;
    PointLinkNode* cutNode = nullptr;  // 裁剪位置
    do
    {
        // 排除当前需要检查的点
        if (curNode != suitNode)
        {
            // 判断当前点是否在形成的三角形中，如果在，则取出一个最靠近x轴，即y最小的
            if (NavMath::IsPointInTriangle(A, B, C, curNode->point))
            {
                if (cutNode == nullptr)
                    cutNode = curNode;
                else if (NavMath::Abs(curNode->point.y - rightInsideNode->point.y)
                    < NavMath::Abs(cutNode->point.y - rightInsideNode->point.y))
                    cutNode = curNode;
            }
        }
        curNode = curNode->nextNode;
    }
    while (curNode != firstNode);
    
    if (cutNode == nullptr)
        cutNode = suitNode;

    // 拷贝 对应的Inside节点插入到当前链表中
    PointLinkNode* copyCutPoint = new PointLinkNode(cutNode->point.x, cutNode->point.y, cutNode->point.z);
    PointLinkNode* copyRightInsidePoint = new PointLinkNode(rightInsideNode->point.x, rightInsideNode->point.y, rightInsideNode->point.z);
    PointLinkNode* cutNextNode = cutNode->nextNode;  // 裁剪的下一个位置，用来接收新的洞的最后一个点
    PointLinkNode* insidePreNode = rightInsideNode->preNode;
    cutNode->nextNode = rightInsideNode;
    rightInsideNode->preNode = cutNode;
    insidePreNode->nextNode = copyRightInsidePoint;
    copyRightInsidePoint->preNode = insidePreNode;
    copyRightInsidePoint->nextNode = copyCutPoint;
    copyCutPoint->preNode = copyRightInsidePoint;
    copyCutPoint->nextNode = cutNextNode;
    cutNextNode->preNode = copyCutPoint;
}

bool PolygonTriangulation::IsPointEar(PointLinkNode* checkNode)
{
    // 首先排除当前不是一个凸边的情况
    if (!checkNode->IsPointConvex())
        return false;
    
    // 检查当前三角形内是否包含其他点
    PointLinkNode* curNode = firstNode;
    if (curNode == nullptr)
        return false;
    
    do
    {
        // 检查当前点是否在需要检查的三角形内
        // 首先排除当前需要检查三角形的三个顶点
        if (checkNode != curNode && checkNode->preNode != curNode && checkNode->nextNode != curNode)
        {
            // 存在点在当前三角形内，当前不是耳尖
            if (NavMath::IsPointInTriangle(
                 checkNode->point,
                 checkNode->nextNode->point,
                 checkNode->preNode->point,
                 curNode->point
                ))
            {
                return false;
            }
        }
        
        curNode = curNode->nextNode;
    }
    while (curNode != firstNode);
    
    return true;
}

bool PolygonTriangulation::OneStepEarClipping()
{
    // 判断当前情况是否可以进行剪裁
    // 检查链表深度
    if (firstNode == nullptr)
        return false;

    // 收入最后一个三角形
    if (firstNode->nextNode->nextNode->nextNode == firstNode)
    {
        triangles.push_back(Triangle(firstNode->point, firstNode->preNode->point, firstNode->nextNode->point));
        firstNode = nullptr;
        return false;
    }
    
    // 取当前第一个耳尖，进行剪裁
    PointLinkNode* curNode = firstNode;
    do
    {
        // 针对当前行为进行剪裁
        if (IsPointEar(curNode))
        {
            // 如果当前裁剪的点是头结点，则将头结点交给下一个节点
            if (curNode == firstNode)
                firstNode = curNode->nextNode;
            
            // 在链表中去除当前点
            curNode->preNode->nextNode = curNode->nextNode;
            curNode->nextNode->preNode = curNode->preNode;
            // 追加分割好的三角形
            triangles.push_back(Triangle(curNode->point, curNode->preNode->point, curNode->nextNode->point));
            return true;
        }
        
        curNode = curNode->nextNode;
    } while(curNode != firstNode);
    
    return false;
}



