#pragma once
#include <vector>

#include "Vectors.h"

namespace ZXNavMesh
{
    class NavMeshHelper
    {
    public:
        // 判断当前是否为外部轮廓
        static bool IsTriangleOutside(std::vector<Vector3> points)
        {
            double S = 0;
            
            int n = points.size();
            for (int i = 0; i < n - 1; i++) {
                const Vector3 point = points[i];
                const Vector3 nextPoint = points[i + 1];
                S += -0.5 * (nextPoint.y + point.y) * (nextPoint.x - point.x);
            }
            S += -0.5 * (points[0].y + points[n - 1].y) * (points[0].x - points[n - 1].x);
            return S < 0;
        }
    };

}