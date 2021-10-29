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

    std::vector<Path<double>> PolygonNavMeshTool::GenFinalTriangles()
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
        return pathsD.data;
    }

    vector<ClipTriangle*> PolygonNavMeshTool::FindPath(Vector3 start, Vector3 end)
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
                if (startTriangle == nullptr && triangle->IsPointInTriangle(start))
                {
                    startTriangle = triangle;
                    pathTriangles.push_back(triangle);
                }
                
                if (endTriangle == nullptr && triangle->IsPointInTriangle(end))
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
            }
        }
        
        return pathTriangles;
    }
}
