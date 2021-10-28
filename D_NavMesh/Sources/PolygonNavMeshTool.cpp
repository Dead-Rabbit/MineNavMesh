#include "PolygonNavMeshTool.h"

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
        for (vector<ClipTriangle*> triangles : genTriangleGroups)
        {
            for (ClipTriangle* triangle : triangles)
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
            
            // 看看起始三角形是否存在
            if (startTriangle != nullptr && endTriangle != nullptr)
            {
                break;
            }
        }
        
        // 看看起始三角形是否存在
        if (startTriangle == nullptr && endTriangle == nullptr)
        {
            return pathTriangles;
        }

        // dfs 查找最短路径
        // startTriangle->GetLinkedClipTriangles()
        
        return pathTriangles;
    }
}
