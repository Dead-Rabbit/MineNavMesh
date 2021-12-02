#include "PolygonTriangulation.h"

#include <algorithm>

namespace PolygonNavMesh{

    int ClipTriangle::triangleNum = 0;
    int PointLinkNode::pointNum = 0;
    int OutsidePolygon::polygonNum = 0;

    void PolygonTriangulation::AddPolygonOutPoints(std::vector<NavMeshBase::Vector3> edgePoints)
    {
        // 检查当前是否为轮廓
        OutsidePolygon* newPolygon = new OutsidePolygon(edgePoints);
        // 循环未归位岛洞，检查是否所属当前轮廓
        auto insidePathIt = insidePolygons.begin();
        while(insidePathIt != insidePolygons.end())
        {
            auto insidePathNodeIt = insidePathIt->begin();
            if (newPolygon->IsPointInPolygon(insidePathNodeIt[0]))
            {
                // 将岛洞加入到Polygon中，并删除当前记录
                newPolygon->AddPolygonInsidePoints(*insidePathIt);
                insidePolygons.erase(insidePathIt);
                break;
            }
            ++insidePathIt;
        }
        polygons.push_back(newPolygon);
    }

    std::vector<std::vector<ClipTriangle*>> PolygonTriangulation::GetGenTriangles() const
    {
        std::vector<std::vector<ClipTriangle*>> triangleGroups;
        for (const auto polygon : polygons)
        {
            triangleGroups.push_back(polygon->GetGenTriangles());
        }
        return triangleGroups;
    }

    void PolygonTriangulation::AddPolygonInsidePoints(std::vector<NavMeshBase::Vector3> insidePoints)
    {
        // 检查当前轮廓是否有包含当前岛洞的
        for (int i = 0; i < polygons.size(); ++i)
        {
            const auto outsidePolygon = polygons[i];
            if (outsidePolygon->IsPointInPolygon(insidePoints[0]))
            {
                outsidePolygon->AddPolygonInsidePoints(insidePoints);
                return;
            }
        }
        
        insidePolygons.push_back(insidePoints);
    }

    bool PolygonTriangulation::OneStepEarClipping()
    {
        bool ifExist = false;
        for (int i = 0;i < polygons.size(); i++)
        {
            ifExist = polygons[i]->OneStepEarClipping() || ifExist;
        }
        return ifExist;
    }

    OutsidePolygon::OutsidePolygon(std::vector<NavMeshBase::Vector3> edgePoints)
    {
        // 设置边框时，重置所有内容
        Reset();
        
        this->num = ++polygonNum;
        this->edgePoints = edgePoints;
        // 生成链表
        const int pointSize = edgePoints.size();
        PointLinkNode* curNode = firstNode;
        for (int i = 0; i < pointSize; i++)
        {
            const auto point = edgePoints[i];

            // 插入链表节点
            if (firstNode == nullptr)
            {
                firstNode = new PointLinkNode(point.x, point.y, point.z);
                curNode = firstNode;
                continue;
            }

            auto newPoint = new PointLinkNode(point.x, point.y, point.z);
            curNode->nextNode = newPoint;
            PointLinkNode* preNode = curNode;
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

    void OutsidePolygon::AddPolygonInsidePoints(std::vector<NavMeshBase::Vector3> innerPoints)
    {
        // 生成新的链表
        insidePointsList.push_back(innerPoints);
        
        const int pointSize = innerPoints.size();
        PointLinkNode* firstInsideNode = nullptr;
        PointLinkNode* curInsideNode = firstInsideNode;
        PointLinkNode* rightNode = nullptr;
        for (int i = 0; i < pointSize; i++)
        {
            const auto point = innerPoints[i];

            // 插入链表节点
            if (firstInsideNode == nullptr)
            {
                firstInsideNode = new PointLinkNode(point.x, point.y, point.z);
                curInsideNode = firstInsideNode;
                rightNode = firstInsideNode;
                continue;
            }
            
            curInsideNode->nextNode = new PointLinkNode(point.x, point.y, point.z);
            PointLinkNode* preInsideNode = curInsideNode;
            curInsideNode = curInsideNode->nextNode;
            curInsideNode->preNode = preInsideNode;

            // 比较查出当前最右点
            if (curInsideNode->point.x > rightNode->point.x)
                rightNode = curInsideNode;
        }
        curInsideNode->nextNode = firstInsideNode;
        firstInsideNode->preNode = curInsideNode;

        // 考虑使用最右点进行排序，将右侧点x值最大的放到前面
        auto nodesIt = insideFirstNodes.begin();
        bool isInsert = false;
        while(nodesIt != insideFirstNodes.end())
        {
            if ((*nodesIt)->point.x < rightNode->point.x)
            {
                nodesIt = insideFirstNodes.insert(nodesIt, rightNode);
                isInsert = true;
                break;
            }
            ++nodesIt;
        }
        
        if (!isInsert)
        {
            insideFirstNodes.push_back(rightNode);
        }
    }

    void OutsidePolygon::ApplyInsidePolygonPoints()
    {
        if (_beginEarClipping)
            return;
        
        _beginEarClipping = true;
        
        for(int nodePairIndex = 0; nodePairIndex < insideFirstNodes.size(); nodePairIndex++)
        {
            // 寻找最靠右的 Inside 点
            PointLinkNode* rightInsideNode = insideFirstNodes[nodePairIndex];
            
            // 获取从inner右点触发向x轴正方向走的最大线段
            const NavMeshBase::Vector3 compareLineStart = rightInsideNode->point;
            const NavMeshBase::Vector3 compareLineEnd = NavMeshBase::Vector3(rightEdgeNode->point.x + 10, rightInsideNode->point.y, 0);

            PointLinkNode* suitNode = nullptr;
            NavMeshBase::Vector3 suitCrossPoint;
            // 判断靠右内部节点与哪个线段的相交点最靠近y轴
            PointLinkNode* curNode = firstNode;
            if (curNode == nullptr)
                return ;
            do
            {
                NavMeshBase::Vector3 crossPoint;
                if (NavMeshBase::NavMath::GetSegmentLinesIntersection(compareLineStart, compareLineEnd,
                    curNode->point, curNode->nextNode->point, crossPoint))
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
            const NavMeshBase::Vector3 A = rightInsideNode->point, B = suitCrossPoint, C = suitNode->point;
            PointLinkNode* cutNode = nullptr;  // 裁剪位置
            do
            {
                // 排除当前需要检查的点
                if (curNode != suitNode)
                {
                    // 判断当前点是否在形成的三角形中，如果在，则取出一个最靠近x轴，即y最小的
                    if (NavMeshBase::NavMath::IsPointInTriangle(A, B, C, curNode->point, false))
                    {
                        if (cutNode == nullptr)
                            cutNode = curNode;
                        else if (NavMeshBase::NavMath::Abs(curNode->point.y - rightInsideNode->point.y)
                            < NavMeshBase::NavMath::Abs(cutNode->point.y - rightInsideNode->point.y))
                            cutNode = curNode;
                    }
                }
                curNode = curNode->nextNode;
            }
            while (curNode != firstNode);
            
            if (cutNode == nullptr)
                cutNode = suitNode;

            // 拷贝 对应的Inside节点插入到当前链表中
            PointLinkNode* copyCutPoint = cutNode->CopyNode();
            PointLinkNode* copyRightInsidePoint = rightInsideNode->CopyNode();
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
    }

    ClipTriangle::ClipTriangle(PointLinkNode* A, PointLinkNode* B, PointLinkNode* C, int numInPolygon)
    {
        this->A = A;
        this->B = B;
        this->C = C;
        this->num = ++triangleNum;
        this->numInPolygon = numInPolygon;

        // 按照逆时针顺序存入
        this->points.push_back(this->A);
        this->points.push_back(this->C);
        this->points.push_back(this->B);

        // 计算当前内心
        // centerPos = NavMeshBase::NavMath::CalculateInsideCenter(this->A->point, this->B->point, this->C->point);
        // 计算当前质心
        centerPos = (this->A->point + this->B->point + this->C->point) / 3;
        
        // 将三角形push到三个点中
        this->A->AddLinkTriangle(this);
        this->B->AddLinkTriangle(this);
        this->C->AddLinkTriangle(this);
    }

    std::vector<std::pair<ClipTriangle*, ClipLine*>> ClipTriangle::GetLinkedClipTriangles()
    {
        if (!InitLinkedTriangle)
        {
            // 遍历获取当前所有点及其影子点的链接三角形，处当前三角形外
            for (PointLinkNode* edgePoint : points)
            {
                // 检查轮廓节点
                GetLinkedClipTrianglesByPoint(edgePoint);
                // 检查影子节点
                if (edgePoint->linkNode)
                    GetLinkedClipTrianglesByPoint(edgePoint->linkNode);
            }
            InitLinkedTriangle = false;
        }
        
        return linkedTriangles;
    }

    void ClipTriangle::GetLinkedClipTrianglesByPoint(PointLinkNode* point)
    {
        for (ClipTriangle* otherTriangle : point->linkTriangles)
        {
            // 检查其他的三角形是否与当前三角形相交
            ClipLine* clipLine = new ClipLine;
            if (otherTriangle != this && IsTriangleLink(otherTriangle, clipLine))
            {
                // 将三角形push到结果中，检查当前三角形是否已放入
                auto it = std::find_if(linkedTriangles.begin(), linkedTriangles.end(),
                    [otherTriangle](std::pair<ClipTriangle*, ClipLine*> contentTriPair)
                    {
                        return contentTriPair.first == otherTriangle;
                    });
                if (it == linkedTriangles.end())
                {
                    linkedTriangles.push_back(std::pair<ClipTriangle*, ClipLine*>(otherTriangle, clipLine));
                }
            }
        }
    }

    bool ClipTriangle::IsTriangleLink(const ClipTriangle* otherTriangle, ClipLine* outputLine)
    {
        // 两个三角形相连接的点的数量
        int connectPointNum = 0;
        int preLinkIndex = 0;
        std::vector<PointLinkNode*> matchedPoints;
        for (PointLinkNode* otherEdgePoint : otherTriangle->points)
        {
            // otherPoint 为其他三角形的边框点
            for(int i = 0; i < points.size(); i++)
            {
                PointLinkNode* edgePoint = points[i];
                // 计算到“穿出边”时，需要判断两个三角形是怎么形成的临近
                // 考虑一种情况为 另外一个三角形的两个点和当前三角形的两个点是通过拷贝形成的
                if (edgePoint == otherEdgePoint || edgePoint == otherEdgePoint->linkNode)
                {
                    // 按照三角形中
                    matchedPoints.push_back(edgePoint);
                    connectPointNum++;
                    if (outputLine->A == nullptr)
                    {
                        outputLine->A = edgePoint;
                        preLinkIndex = i;
                    } else
                    {
                        const int interval = i - preLinkIndex;
                        if (interval == 1 || interval == -2)
                            outputLine->B = edgePoint;
                        else if (interval == 2 || interval == -1)
                        {
                            outputLine->B = outputLine->A;
                            outputLine->A = edgePoint;
                        }
                    }
                    break;
                }
            }
        }
        
        return connectPointNum == 2;
    }

    bool ClipTriangle::IsPointInTriangle(NavMeshBase::Vector3 point, double diff)
    {
        if (A->point.x == B->point.x && A->point.x == C->point.x ||
            A->point.y == B->point.y && A->point.y == C->point.y)
            return false;
        
        return NavMeshBase::NavMath::IsPointInTriangle(A->point, B->point, C->point, point, true, diff);
    }

    bool OutsidePolygon::IsPointEar(PointLinkNode* checkNode) const
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
                if (NavMeshBase::NavMath::IsPointInTriangle(
                     checkNode->point,
                     checkNode->nextNode->point,
                     checkNode->preNode->point,
                     curNode->point, false
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

    bool OutsidePolygon::OneStepEarClipping()
    {
        // 进行岛洞排序
        if (!_beginEarClipping)
            ApplyInsidePolygonPoints();
        
        // 判断当前情况是否可以进行剪裁
        // 检查链表深度
        if (firstNode == nullptr)
            return false;

        // 收入最后一个三角形
        if (firstNode->nextNode->nextNode->nextNode == firstNode)
        {
            // 排除三个点的x或y相通的情况
            triangles.push_back(new ClipTriangle(firstNode, firstNode->preNode, firstNode->nextNode, triangles.size()));
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
                ClipTriangle* clipTriangle = new ClipTriangle(curNode, curNode->preNode, curNode->nextNode, triangles.size());
                triangles.push_back(clipTriangle);
                
                return true;
            }
            
            curNode = curNode->nextNode;
        } while(curNode != firstNode);

        return false;
    }

    bool OutsidePolygon::IsPointInPolygon(NavMeshBase::Vector3 point) const
    {
        return NavMeshBase::NavMath::IsPointInPolygonByRayCast(edgePoints, point);
    }

    bool OutsidePolygon::GetNearCrossFromOutsidePoint(NavMeshBase::Vector3 point, NavMeshBase::Vector3& nearPoint)
    {
        bool findPoint = false;
        double minDis = 0;
        const int pointSize = edgePoints.size();
        for (int i = 0; i < pointSize; i++)
        {
            const auto curPoint = edgePoints[i];
            const auto nextPoint = edgePoints[i + 1 < pointSize ? i + 1 : 0];
            NavMeshBase::Vector3 nearestPoint;
            const double tempDis = NavMeshBase::NavMath::PointToSegDist(point, curPoint, nextPoint, nearestPoint);
            if (!findPoint || tempDis < minDis)
            {
                findPoint = true;
                minDis = tempDis;
                nearPoint = nearestPoint;
            }
        }

        return findPoint;
    }

    bool OutsidePolygon::GetNearCrossFromInsidePoint(NavMeshBase::Vector3 point, NavMeshBase::Vector3& nearPoint)
    {
        bool findPoint = false;
        double minDis = 0;
        for (int j = 0; j < insidePointsList.size(); j++)
        {
            auto insidePoints = insidePointsList[j];
            const int pointSize = insidePoints.size();
            for (int i = 0; i < pointSize; i++)
            {
                const auto curPoint = insidePoints[i];
                const auto nextPoint = insidePoints[i + 1 < pointSize ? i + 1 : 0];
                NavMeshBase::Vector3 nearestPoint;
                const double tempDis = NavMeshBase::NavMath::PointToSegDist(point, curPoint, nextPoint, nearestPoint);
                if (!findPoint || tempDis < minDis)
                {
                    findPoint = true;
                    minDis = tempDis;
                    nearPoint = nearestPoint;
                }
            }
        }
        
        return findPoint;
    }
}
