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

    vector<Vector3> PolygonNavMeshTool::FindPath(Vector3 startPoint, Vector3 endPoint, vector<Vector3> &pathBeforeSmooth)
    {
        vector<ClipTriangle*> pathTriangles = vector<ClipTriangle*>();
        
        // 获取生成的多个三角形组
        vector<vector<ClipTriangle*>> genTriangleGroups = triangulationTool.GetGenTriangles();

        ClipTriangle* startTriangle = nullptr;
        ClipTriangle* endTriangle = nullptr;
        // 检查起始点和结束点所在的三角形
        for (vector<ClipTriangle*> triangleGroup : genTriangleGroups)
        {
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
            if (startTriangle == nullptr && endTriangle == nullptr)
            {
            
            } else if(startTriangle != nullptr && endTriangle == nullptr)
            {
                // TODO 判断起点在轮廓里，终点不在轮廓里的情况
            
            } else if (startTriangle != nullptr && endTriangle != nullptr)
            {
                // Dijkstra 查找最短路径，构建三角形图
                Graph_DG graph = Graph_DG();
                graph.createGraph(triangleGroup);
                graph.Dijkstra(startTriangle);
                pathTriangles = graph.find_path_triangles(endTriangle);

                // 获得最终路径三角形，进行路径平滑，此处使用拐角点法
                // 参考 https://blog.csdn.net/liqiang981/article/details/70207912
                {
                    cout << "-------------------->>>" << endl;
                    
                    ClipTriangle* preTriangle = nullptr;
                    Vector3 leftPoint;                  // 当前记录的出口线的左点
                    Vector3 rightPoint;                 // 当前记录的出口线的右点
                    vector<Vector3> finalPath;          // 最终输出路径
                    finalPath.push_back(startPoint);
                    pathBeforeSmooth.clear();
                    pathBeforeSmooth.push_back(startPoint);
                    Vector3 curPoint = startPoint;  // 当前节点
                    vector<ClipLine*> pathLines;
                    // 获取所有路径穿出口
                    for (int i = 0; i < pathTriangles.size(); i++)
                    {
                        const auto curTriangle = pathTriangles[i];
                        
                        pathBeforeSmooth.push_back(curTriangle->centerPos);
                        
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
                                break;
                            }
                        }
                        preTriangle = curTriangle;
                    }

                    bool recordPoint = true;
                    // 遍历所有相交线，获得最终路径
                    for(int i = 0; i < pathLines.size(); i++)
                    {
                        ClipLine* clipLine = pathLines[i];
                        
                        Vector3 lineRight = clipLine->A->point;
                        Vector3 lineLeft = clipLine->B->point;
                        
                        if (recordPoint)
                        {
                            leftPoint = lineLeft;
                            rightPoint = lineRight;
                            recordPoint = false;
                            continue;
                        }

                        // 检查left 或 right 是否越界, z 表达为，负数在Left，正数在Right
                        Vector3 leftCrossEndRes = (lineLeft - curPoint).crossProduct(endPoint - curPoint);
                        Vector3 rightCrossEndRes = (lineRight - curPoint).crossProduct(endPoint - curPoint);
                        // 判断左点的情况
                        Vector3 leftCrossLeftRes = (leftPoint - curPoint).crossProduct(lineLeft - curPoint);
                        Vector3 rightCrossLeftRes = (rightPoint - curPoint).crossProduct(lineLeft - curPoint);
                        if (leftCrossLeftRes.z > 0)
                        {
                            if (rightCrossLeftRes.z < 0)
                            {
                                leftPoint = lineLeft;
                                cout << "Left inside" << endl;
                            } else
                            {
                                // 当前情况为，线的left节点在目前right节点的right或直线上
                                // 则记录当前right点为节点，并将left 和 right 节点置为当前线上的节点
                                cout << "Left outside Of Right" << endl;
                                finalPath.push_back(rightPoint);
                                curPoint = rightPoint;
                                // leftPoint = lineLeft;
                                // rightPoint = lineRight;
                                recordPoint = true;
                            }
                        }
                        
                        // 判断右点情况
                        Vector3 leftCrossRightRes = (leftPoint - curPoint).crossProduct(lineRight - curPoint);
                        Vector3 rightCrossRightRes = (rightPoint - curPoint).crossProduct(lineRight - curPoint);
                        if (rightCrossRightRes.z < 0)
                        {
                            if (leftCrossRightRes.z > 0)
                            {
                                rightPoint = lineRight;
                                cout << "Right inside" << endl;
                            } else
                            {
                                // 当前情况为，线的Right节点在目前left节点的left或直线上
                                // 则记录当前left点为节点，并将left 和 right 节点置为当前线上的节点
                                cout << "Right outside" << endl;
                                
                                finalPath.push_back(leftPoint);
                                curPoint = leftPoint;
                                // leftPoint = lineLeft;
                                // rightPoint = lineRight;
                                recordPoint = true;
                            }
                        }
                    }
                    
                    // 判断左点的情况
                    Vector3 leftCrossEndRes = (leftPoint - curPoint).crossProduct(endPoint - curPoint);
                    Vector3 rightCrossEndRes = (rightPoint - curPoint).crossProduct(endPoint - curPoint);
                    cout << leftCrossEndRes.z << " - " << rightCrossEndRes.z << endl;
                    if (rightCrossEndRes.z >= 0)
                    {
                        // 追加右侧侧的遗留内容
                        finalPath.push_back(rightPoint);
                    }
    
                    if (leftCrossEndRes.z <= 0)
                    {
                        // 追加左侧的遗留内容
                        finalPath.push_back(leftPoint);
                    }

                    finalPath.push_back(endPoint);
                    pathBeforeSmooth.push_back(endPoint);
                    return finalPath;
                }
            }
        }
        return {};
    }
}
