﻿#include "PolygonTriangulation.h"

namespace ZXNavMesh{

    int ClipLine::lineNum = 0;
    int ClipTriangle::triangleNum = 0;
    int PointLinkNode::pointNum = 0;
    int OutsidePolygon::polygonNum = 0;

    void PolygonTriangulation::AddPolygonOutPoints(vector<Vector3> edgePoints)
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

    void PolygonTriangulation::AddPolygonInsidePoints(vector<Vector3> insidePoints)
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

    OutsidePolygon::OutsidePolygon(vector<Vector3> edgePoints)
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
            
            curNode->nextNode = new PointLinkNode(point.x, point.y, point.z);
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

    void OutsidePolygon::AddPolygonInsidePoints(vector<Vector3> innerPoints)
    {
        // 生成新的链表
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
            const Vector3 compareLineStart = rightInsideNode->point;
            const Vector3 compareLineEnd = Vector3(rightEdgeNode->point.x + 10, rightInsideNode->point.y, 0);

            PointLinkNode* suitNode = nullptr;
            Vector3 suitCrossPoint;
            // 判断靠右内部节点与哪个线段的相交点最靠近y轴
            PointLinkNode* curNode = firstNode;
            if (curNode == nullptr)
                return ;
            do
            {
                Vector3 crossPoint;
                if (NavMath::GetSegmentLinesIntersection(compareLineStart, compareLineEnd,
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
            const Vector3 A = rightInsideNode->point, B = suitCrossPoint, C = suitNode->point;
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

    ClipTriangle::ClipTriangle(PointLinkNode* A, PointLinkNode* B, PointLinkNode* C)
    {
        this->A = A;
        this->B = B;
        this->C = C;
        this->num = ++triangleNum;

        centerPos = NavMath::CalculateInsideCenter(this->A->point, this->B->point, this->C->point);

        // // 查找对应点的线段，如果不存在则创建
        // CreateTriangleLine(this->A, this->B);
        // CreateTriangleLine(this->B, this->C);
        // CreateTriangleLine(this->C, this->A);
    }

    // ClipLine* ClipTriangle::CreateTriangleLine(PointLinkNode* pA, PointLinkNode* pB)
    // {
    //     // 寻找两点构成的直线
    //     ClipLine* lineAB = pA->GetLineByOtherNode(pB);
    //     if (lineAB == nullptr)
    //     {
    //         lineAB = new ClipLine(pA, pB);
    //         
    //         // 创建并放到各自的点上
    //         std::cout << "创建线段" << lineAB->num << " " << pA->num << "-" << pB->num << endl;
    //         
    //         pA->AddLine(lineAB);
    //         pB->AddLine(lineAB);
    //         // 将直线追加到当前三角形记录的line中
    //         lines.push_back(lineAB);
    //     }
    //     
    //     // 新增记录在Line中当前三角形的指针
    //     auto lineTriangles = lineAB->triangles;
    //     const auto findIt = std::find_if(lineTriangles.begin(), lineTriangles.end(),
    //         [this](ClipTriangle* triangle)
    //         {
    //             return this == triangle;
    //         });
    //     // 没有找到，则在线上新增当前三角形
    //     if (findIt == lineTriangles.end())
    //     {
    //         std::cout << "追加三角形" << this->num << " 到线段" << lineAB->num << "上" << endl;
    //         lineAB->triangles.push_back(this);
    //     }
    //     
    //     return lineAB;
    // }

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
            triangles.push_back(new ClipTriangle(firstNode, firstNode->preNode, firstNode->nextNode));
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
                ClipTriangle* clipTriangle = new ClipTriangle(curNode, curNode->preNode, curNode->nextNode);
                triangles.push_back(clipTriangle);
                
                return true;
            }
            
            curNode = curNode->nextNode;
        } while(curNode != firstNode);

        return false;
    }

    bool OutsidePolygon::IsPointInPolygon(Vector3 point) const
    {
        return NavMath::IsPointInPolygonByRayCast(edgePoints, point);
    }
}