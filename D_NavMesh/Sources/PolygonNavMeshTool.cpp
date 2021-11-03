#include "PolygonNavMeshTool.h"

#include "Dijkstra.h"

namespace PolygonNavMesh
{
    void PolygonNavMeshTool::AddPolygonOutsideContour(vector<Vector3> contour)
    {
        Path<double> subjectPath = Path<double>();
        for (const Vector3 contourPoint : contour)
        {
            subjectPath.push_back(Point<double>(contourPoint.x, contourPoint.y));
        }
        clipperD.AddPath(subjectPath, PathType::Subject, false);
    }

    void PolygonNavMeshTool::AddPolygonInsideContour(vector<Vector3> contour)
    {
        Path<double> clipPath = Path<double>();
        for (const Vector3 contourPoint : contour)
        {
            clipPath.push_back(Point<double>(contourPoint.x, contourPoint.y));
        }
        clipperD.AddPath(clipPath, PathType::Clip, false);
    }

    vector<vector<ClipTriangle*>> PolygonNavMeshTool::GenerateFinalTriangles()
    {
        // 裁剪后输出的Path数据
        PathsD pathsD = PathsD();
        // 获取最后输出path
        if (clipperD.Execute(ClipType::Difference, FillRule::Negative, pathsD))
        {
            std::vector<Path<double>> resultPaths = pathsD.data;

            // 将resultPaths 加入到三角化程序中
            // 获取所有路径点和对应的岛洞
            for(int i = 0; i < resultPaths.size(); i++)
            {
                auto path = resultPaths[i];
                path.Reverse();
            
                vector<Vector3> pathNodes;
                for (auto pathNode : path.data)
                {
                    pathNodes.push_back(Vector3(pathNode.x, pathNode.y, 0));
                }
                // 判断当前线段是否为外边框和岛洞
                triangulationTool.AddPolygonPoints(pathNodes);
            }

            // 三角化，并连接三角形之间的联系
            triangulationTool.EarClipping();

            // TODO 考虑三角化后，生成搜索用 图 组
        }
        return triangulationTool.GetGenTriangles();
    }

    vector<Vector3> PolygonNavMeshTool::FindPath(Vector3 startPoint, Vector3 endPoint)
    {
        vector<ClipTriangle*> pathTriangles = vector<ClipTriangle*>();
        
        // 获取生成的多个三角形组
        vector<vector<ClipTriangle*>> genTriangleGroups = triangulationTool.GetGenTriangles();

        // 检查起始点和结束点所在的三角形
        for (vector<ClipTriangle*> triangleGroup : genTriangleGroups)
        {
            ClipTriangle* startTriangle = nullptr;
            ClipTriangle* endTriangle = nullptr;
            // 在一个三角形组合内搜寻
            for (ClipTriangle* triangle : triangleGroup)
            {
                if (startTriangle == nullptr && triangle->IsPointInTriangle(startPoint))
                {
                    startTriangle = triangle;
                    pathTriangles.push_back(triangle);
                }
                
                if (endTriangle == nullptr && triangle->IsPointInTriangle(endPoint))
                {
                    endTriangle = triangle;
                    pathTriangles.push_back(triangle);
                }
            }
            
            // TODO 两个点都不在这个轮廓中
            if(startTriangle != nullptr && endTriangle == nullptr)
            {
                // 当前起点在轮廓里，终点不在轮廓里，目标修改为当前轮廓的最靠近终点的点
                
            } else if (startTriangle != nullptr && endTriangle != nullptr)
            {
                // Dijkstra 查找最短路径，构建三角形图
                Graph_DG graph = Graph_DG();
                graph.createGraph(triangleGroup);
                graph.Dijkstra(startTriangle);
                pathTriangles = graph.find_path_triangles(endTriangle);

                // 获得最终路径三角形，进行路径平滑，此处使用拐角点法
                ClipTriangle* preTriangle = nullptr;
                vector<ClipLine*> pathLines;
                // std::cout << "Vector3 startPoint  = Vector3(" << startPoint.x << ", " << startPoint.y << ", 0);" << endl;
                // std::cout << "Vector3 endPoint    = Vector3(" << endPoint.x << ", " << endPoint.y << ", 0);" << endl;
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
                    vector<pair<ClipTriangle*, ClipLine*>> preLinkTrianglePairs = preTriangle->GetLinkedClipTriangles();
                    
                    for (int j = 0; j < preLinkTrianglePairs.size(); j++)
                    {
                        pair<ClipTriangle*, ClipLine*> preLinkPair = preLinkTrianglePairs[j];
                        // 是否为链接的当前位置
                        if (preLinkPair.first == curTriangle)
                        {
                            const auto clipLine = preLinkPair.second;
                            pathLines.push_back(clipLine);
                            // std::cout << "pathLines.push_back(Line(Vector3" << clipLine->A->point
                            // << ",Vector3" << clipLine->B->point << "));" << endl;
                            break;
                        }
                    }
                    preTriangle = curTriangle;
                }

                vector<Vector3> finalPath;          // 最终输出路径
                finalPath.push_back(startPoint);
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
                Vector3 curPoint = startPoint;  // 当前节点
                Vector3 rightLeg = pathLines[0]->A->point - curPoint;
                Vector3 leftLeg = pathLines[0]->B->point - curPoint;
                auto curPointIndex = 0, leftLegIndex = 0, rightLegIndex = 0;
                for(int i = 0; i < pathLines.size(); i++)
                {
                    auto curLine = pathLines[i];

                    auto newRightLeg = curLine->A->point - curPoint;
                    auto cpTightenFunnel = newRightLeg.crossProduct(rightLeg);
                    if (cpTightenFunnel.z >= 0.0f)
                    {
                        auto cpDenegrateFunnel = newRightLeg.crossProduct(leftLeg);
                        if (cpDenegrateFunnel.z < 0.0f) //No overlap, tighten!
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
                        if (cpDenegrateFunnel.z > 0.0f) //No overlap, tighten!
                        {
                            leftLeg = newLeftLeg;
                            leftLegIndex = i;
                        }
                        else
                        {
                            //Rightleg becomes new curPoint point
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
                Vector3 prePoint = finalPath[0];
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
        }
        return {};
    }
}
