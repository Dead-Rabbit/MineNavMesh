#include "PolygonTriangulation.h"

void PolygonTriangulation::SetPolygonPoints(vector<Vector3> edgePoints)
{
    this->edgePoints = edgePoints;
    // 生成链表
    const int pointSize = edgePoints.size();
    PointLinkNode* curNode = firstNode;
    PointLinkNode* preNode = firstNode;
    for (int i = 0; i < pointSize; i++)
    {
        if (firstNode == nullptr)
        {
            firstNode = new PointLinkNode(i+1, edgePoints[i]);
            curNode = firstNode;
            continue;
        }
        
        curNode->nextNode = new PointLinkNode(i+1, edgePoints[i]);
        preNode = curNode;
        curNode = curNode->nextNode;
        curNode->preNode = preNode;
    }
    curNode->nextNode = firstNode;
    firstNode->preNode = curNode;
    
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
                    return false;
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

vector<Vector3> PolygonTriangulation::GetValidPoints() const
{
    vector<Vector3> validPoints;
            
    PointLinkNode* outCurNode = firstNode;
    if (outCurNode == nullptr)
        return validPoints;
            
    do {
        validPoints.push_back(outCurNode->point);
        outCurNode = outCurNode->nextNode;
    } while(outCurNode != firstNode);
    return validPoints;
}


