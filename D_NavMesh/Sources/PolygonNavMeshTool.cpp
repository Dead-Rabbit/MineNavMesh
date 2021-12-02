#include "PolygonNavMeshTool.h"

#include "Dijkstra.h"

namespace PolygonNavMesh
{
    void PolygonNavMeshTool::AddPolygonOutsideContour(std::vector<NavMeshBase::Vector3> contour)
    {
        clipperlib::Path<double> subjectPath = clipperlib::Path<double>();
        for (const NavMeshBase::Vector3 contourPoint : contour)
        {
            subjectPath.push_back(clipperlib::Point<double>(contourPoint.x, contourPoint.y));
        }
        subjectPaths.push_back(subjectPath);
        genTriangleDirty = true;
    }

    int PolygonNavMeshTool::AddPolygonInsideContour(std::vector<NavMeshBase::Vector3> contour)
    {
        clipperlib::Path<double> clipPath = clipperlib::Path<double>();
        for (const NavMeshBase::Vector3 contourPoint : contour)
        {
            clipPath.push_back(clipperlib::Point<double>(contourPoint.x, contourPoint.y));
        }
        
        // 追加到工具记录的孔洞轮廓中
        size_t newIndex = clipPathMap.size();
        const size_t _removedPathNum = _RemovedClipPathIndex.size();
        if (_removedPathNum > 0)
        {
            newIndex = _RemovedClipPathIndex[_removedPathNum - 1];
            _RemovedClipPathIndex.pop_back();
        }
        clipPathMap[newIndex] = clipPath;
        genTriangleDirty = true;
        
        return newIndex;
    }

    bool PolygonNavMeshTool::RemovePolygonInsideContour(int contourIndex)
    {
        const auto find_it = clipPathMap.find(contourIndex);
        if (find_it == clipPathMap.end())
            return false;

        clipPathMap.erase(find_it);
        
        // 检查在删除列表中是否存在，如果存在则不录入
        const auto add_it = std::find(_RemovedClipPathIndex.begin(), _RemovedClipPathIndex.end(), contourIndex);
        if (add_it == _RemovedClipPathIndex.end())
            _RemovedClipPathIndex.push_back(contourIndex);

        genTriangleDirty = true;
        return true;
    }

    std::vector<std::vector<ClipTriangle*>> PolygonNavMeshTool::GenerateFinalTriangles()
    {
        if (!genTriangleDirty)
            return triangulationTool.GetGenTriangles();

        triangulationTool.Reset();
        // Vattis 轮廓切割
        clipperlib::ClipperD clipperD = clipperlib::ClipperD();
        for (auto subjectPath : subjectPaths)
        {
            clipperD.AddPath(subjectPath, clipperlib::PathType::Subject, false);
        }
        for (auto clip_it = clipPathMap.begin(); clip_it != clipPathMap.end(); ++clip_it)
        {
            clipperD.AddPath(clip_it->second, clipperlib::PathType::Clip, false);
        }
        
        // 裁剪后输出的Path数据
        clipperlib::PathsD pathsD = clipperlib::PathsD();
        // 获取最后输出path
        if (clipperD.Execute(clipperlib::ClipType::Difference, clipperlib::FillRule::Negative, pathsD))
        {
            std::vector<clipperlib::Path<double>> resultPaths = pathsD.data;

            // 将resultPaths 加入到三角化程序中
            // 获取所有路径点和对应的岛洞
            for(int i = 0; i < resultPaths.size(); i++)
            {
                auto path = resultPaths[i];
                path.Reverse();
            
                std::vector<NavMeshBase::Vector3> pathNodes;
                for (auto pathNode : path.data)
                {
                    pathNodes.push_back(NavMeshBase::Vector3(pathNode.x, pathNode.y, 0));
                }
                // 判断当前线段是否为外边框和岛洞
                triangulationTool.AddPolygonPoints(pathNodes);
            }

            // 三角化，并连接三角形之间的联系
            triangulationTool.EarClipping();
            genTriangleDirty = false;
        }
        return triangulationTool.GetGenTriangles();
    }

    std::vector<NavMeshBase::Vector3> PolygonNavMeshTool::FindPath(NavMeshBase::Vector3 startPoint, NavMeshBase::Vector3 endPoint)
    {
        // 寻路前，尝试根据边框进行三角化，如果之前操作过，则重新生成
        GenerateFinalTriangles();
        
        std::vector<ClipTriangle*> pathTriangles = std::vector<ClipTriangle*>();
        // 获取生成的多个三角形组
        std::vector<OutsidePolygon*> genTrianglePolygons = triangulationTool.GetOutsidePolygons();

        NavMeshBase::Vector3 originStartPoint = startPoint;
        std::vector<NavMeshBase::Vector3> finalPath;          // 最终输出路径
        // 检查起始点和结束点所在的三角形
        ClipTriangle* startTriangle = nullptr;
        ClipTriangle* endTriangle = nullptr;
        bool findStartGroup = false;
        OutsidePolygon* startOutsidePolygon = nullptr;
        OutsidePolygon* endOutsidePolygon = nullptr;
        NavMeshBase::Vector3 * startMiddlePoint = nullptr;
        // 查找起始点和结束点所在的三角组和三角形
        for (OutsidePolygon* outsidePolygon : genTrianglePolygons)
        {
            std::vector<ClipTriangle*> triangleGroup = outsidePolygon->GetGenTriangles();
            
            // 在一个三角形组合内搜寻
            for (ClipTriangle* triangle : triangleGroup)
            {
                if (startTriangle == nullptr && triangle->IsPointInTriangle(startPoint))
                {
                    startTriangle = triangle;
                    startOutsidePolygon = outsidePolygon;
                    findStartGroup = true;
                }
                
                if (endTriangle == nullptr && triangle->IsPointInTriangle(endPoint))
                {
                    endTriangle = triangle;
                    endOutsidePolygon = outsidePolygon;
                }
            }
        }

        // 如果当前起始点在外部，则起始点修改为当前距离最近的一个点
        if (!findStartGroup)
        {
            NavMeshBase::Vector3 minPos;
            double minDis = -1;
            bool findMinPos = false;
            std::vector<OutsidePolygon*> outsidePolygons = triangulationTool.GetOutsidePolygons();
            
            // 首先检查并排除岛洞的情况
            for (auto polygon : outsidePolygons)
            {
                // 点在多边形内
                if (polygon->IsPointInPolygon(startPoint))
                {
                    NavMeshBase::Vector3 nearPoint;
                    if (polygon->GetNearCrossFromInsidePoint(startPoint, nearPoint))
                    {
                        auto tempDis = (nearPoint - startPoint).length();
                        if (minDis < 0 || tempDis < minDis)
                        {
                            findMinPos = true;
                            minDis = tempDis;
                            minPos = nearPoint;
                        }
                    }
                }
            }
            
            // 如果没有找到岛洞的，则检查外边框的
            if (!findMinPos)
            {
                for (auto polygon : outsidePolygons)
                {
                    if (!polygon->IsPointInPolygon(startPoint))
                    {
                        NavMeshBase::Vector3 nearPoint;
                        if (polygon->GetNearCrossFromOutsidePoint(startPoint, nearPoint))
                        {
                            auto tempDis = (nearPoint - startPoint).length();
                            if (minDis < 0 || tempDis < minDis)
                            {
                                findMinPos = true;
                                minDis = tempDis;
                                minPos = nearPoint;
                            }
                        }
                    }
                }
            }
            
            // 找到了最近的点，给start赋值为该点
            if (findMinPos)
            {
                startMiddlePoint = new NavMeshBase::Vector3(minPos.x, minPos.y, minPos.z);
                startPoint = minPos;
                
                for (OutsidePolygon* outsidePolygon : genTrianglePolygons)
                {
                    std::vector<ClipTriangle*> triangleGroup = outsidePolygon->GetGenTriangles();
                    for (ClipTriangle* triangle : triangleGroup)
                    {
                        if (startTriangle == nullptr && triangle->IsPointInTriangle(startPoint, -0.1))
                        {
                            startTriangle = triangle;
                            startOutsidePolygon = outsidePolygon;
                            findStartGroup = true;
                        }
                    }
                }
            }
        } else
        {
            std::cout << "find inside polygon" << std::endl;
        }

        // 找到了起始点和起始组，但是起始点和结束点不在一个组内，则找最靠近结束点的点
        if(findStartGroup && startOutsidePolygon != endOutsidePolygon)
        {
            // 如果当前结束点在目标形状内，则说明在岛洞内
            if (startOutsidePolygon->IsPointInPolygon(endPoint))
            {
                NavMeshBase::Vector3 nearPoint;
                startOutsidePolygon->GetNearCrossFromInsidePoint(endPoint, nearPoint);
                endPoint = nearPoint;
                std::vector<ClipTriangle*> triangleGroup = startOutsidePolygon->GetGenTriangles();
                for (ClipTriangle* triangle : triangleGroup)
                {
                    if (triangle->IsPointInTriangle(endPoint, -0.01))
                    {
                        endTriangle = triangle;
                        endOutsidePolygon = startOutsidePolygon;
                    }
                }
            } else
            {
                NavMeshBase::Vector3 nearPoint;
                startOutsidePolygon->GetNearCrossFromOutsidePoint(endPoint, nearPoint);
                endPoint = nearPoint;
                std::vector<ClipTriangle*> triangleGroup = startOutsidePolygon->GetGenTriangles();
                for (ClipTriangle* triangle : triangleGroup)
                {
                    if (triangle->IsPointInTriangle(endPoint, -0.01))
                    {
                        endTriangle = triangle;
                        endOutsidePolygon = startOutsidePolygon;
                    }
                }
            }
        }

        // 在同一个组内寻路
        if (startTriangle != nullptr && endTriangle != nullptr && startOutsidePolygon != nullptr
            && endOutsidePolygon != nullptr && startOutsidePolygon == endOutsidePolygon)
        {
            // Dijkstra 查找最短路径，构建三角形图
            Graph_DG graph = Graph_DG();
            graph.createGraph(startOutsidePolygon->GetGenTriangles());
            graph.Dijkstra(startTriangle);
            pathTriangles = graph.find_path_triangles(endTriangle);

            // 获得最终路径三角形，进行路径平滑，此处使用拐角点法
            ClipTriangle* preTriangle = nullptr;
            std::vector<ClipLine*> pathLines;
            // 获取所有路径穿出口
            for (int i = 0; i < pathTriangles.size(); i++)
            {
                const auto curTriangle = pathTriangles[i];
                
                if (preTriangle == nullptr)
                {
                    preTriangle = curTriangle;
                    continue;
                }

                // 查找两个三角形交线
                std::vector<std::pair<ClipTriangle*, ClipLine*>> preLinkTrianglePairs = preTriangle->GetLinkedClipTriangles();
                
                for (int j = 0; j < preLinkTrianglePairs.size(); j++)
                {
                    std::pair<ClipTriangle*, ClipLine*> preLinkPair = preLinkTrianglePairs[j];
                    // 是否为链接的当前位置
                    if (preLinkPair.first == curTriangle)
                    {
                        const auto clipLine = preLinkPair.second;
                        pathLines.push_back(clipLine);
                        break;
                    }
                }
                preTriangle = curTriangle;
            }

            finalPath.push_back(originStartPoint);
            if (startMiddlePoint != nullptr)
            {
                finalPath.push_back(*startMiddlePoint);
            }
            if (pathLines.size() == 0)
            {
                finalPath.push_back(endPoint);
                return finalPath;
            }

            ClipLine* finalClipLine = new ClipLine();
            PointLinkNode* endPointLinkNode = new PointLinkNode(endPoint.x, endPoint.y, endPoint.z);
            finalClipLine->A = endPointLinkNode;
            finalClipLine->B = endPointLinkNode;
            pathLines.push_back(finalClipLine);

            // 遍历所有线，将寻路进行平滑操作，此处使用漏斗算法，获得最终路径
            // 参考 https://blog.csdn.net/liqiang981/article/details/70207912
            NavMeshBase::Vector3 curPoint = startPoint;  // 当前节点
            NavMeshBase::Vector3 rightLeg = pathLines[0]->A->point - curPoint;
            NavMeshBase::Vector3 leftLeg = pathLines[0]->B->point - curPoint;
            auto curPointIndex = 0, leftLegIndex = 0, rightLegIndex = 0;
            for(int i = 0; i < pathLines.size(); i++)
            {
                auto curLine = pathLines[i];

                auto newRightLeg = curLine->A->point - curPoint;
                auto cpTightenFunnel = newRightLeg.crossProduct(rightLeg);
                if (cpTightenFunnel.z >= 0.0f)
                {
                    auto cpDenegrateFunnel = newRightLeg.crossProduct(leftLeg);
                    if (cpDenegrateFunnel.z <= 0.0f) //No overlap, tighten!
                    {
                        rightLeg = newRightLeg;
                        rightLegIndex = i;
                    }
                    else
                    {
                        curPoint = curPoint + leftLeg;
                        curPointIndex = leftLegIndex;
                        unsigned int newIt = curPointIndex + 1;
                        leftLegIndex = newIt;
                        rightLegIndex = newIt;
                        i = newIt;

                        //Store point
                        finalPath.push_back(curPoint);

                        //Calculate new legs (if not the end)
                        if (newIt < pathLines.size())
                        {
                            rightLeg = pathLines[rightLegIndex]->A->point - curPoint;
                            leftLeg = pathLines[leftLegIndex]->B->point - curPoint;
                            continue; //Restart
                        }
                    }
                }

                auto newLeftLeg = curLine->B->point - curPoint;
                cpTightenFunnel = newLeftLeg.crossProduct(leftLeg);
                if (cpTightenFunnel.z <= 0.0f) //Move inwards
                {
                    auto cpDenegrateFunnel = newLeftLeg.crossProduct(rightLeg);
                    if (cpDenegrateFunnel.z >= 0.0f) //No overlap, tighten!
                    {
                        leftLeg = newLeftLeg;
                        leftLegIndex = i;
                    }
                    else
                    {
                        //Right Leg becomes new curPoint point
                        curPoint = curPoint + rightLeg;
                        curPointIndex = rightLegIndex;
                        unsigned int newIt = curPointIndex + 1;
                        leftLegIndex = newIt;
                        rightLegIndex = newIt;
                        i = newIt;
                        //Store point
                        finalPath.push_back(curPoint);
                        //Calculate new legs (if not the end)
                        if (newIt < pathLines.size())
                        {
                            //Calculate new legs (if not the end)
                            rightLeg = pathLines[rightLegIndex]->A->point - curPoint;
                            leftLeg = pathLines[leftLegIndex]->B->point - curPoint;
                        }
                    }
                }
            }
            
            finalPath.push_back(endPoint);

            // 优化 final path，将前后两个“相同”的点变为一个点
            NavMeshBase::Vector3 prePoint = finalPath[0];
            auto pointIt = finalPath.begin() + 1;
            while(pointIt != finalPath.end())
            {
                if ((*pointIt - prePoint).squaredLength() < 0.001)
                {
                    pointIt = finalPath.erase(pointIt);
                }
                else
                {
                    prePoint = *pointIt;
                    ++pointIt;
                }
            }
            return finalPath;
        }
        return finalPath;
    }
}
