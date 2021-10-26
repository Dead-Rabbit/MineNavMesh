#pragma once

#include <vector>

#include "../../Base/Graphes.h"
#include "../../Base/Vectors.h"
#include "../../Base/NavMath.h"

using namespace std;

namespace ZXNavMesh
{
    
    // 耳切法最终生成的三角形
    class Triangle
    {
    public:
        Vector3 A, B, C;
        Triangle(Vector3 A, Vector3 B, Vector3 C)
        {
            this->A = A;
            this->B = B;
            this->C = C; 
        }
    };
    
    // 耳切法使用点链表节点
    class PointLinkNode
    {
    public:
        PointLinkNode(double x, double y, double z)
        {
            this->point = Vector3(x, y, z);
            this->num = ++pointNum;
        }

        int num;
	
        // 当前点
        Vector3 point;
        // 双向链
        PointLinkNode* preNode = nullptr;
        PointLinkNode* nextNode = nullptr;

        // 计算当前角是否为凸角
        bool IsPointConvex()
        {
            return NavMath::Cross(
                Vector3(point - preNode->point), Vector3(point - nextNode->point)
            ).z > 0;
        };
    private:
        static int pointNum;
        Line* preLine = nullptr;
        Line* nextLine = nullptr;
    };

    // 耳切法类
    class PolygonTriangulation
    {
    public:
        // 设置需要进行计算的外部节点
        void SetPolygonOutPoints(vector<Vector3> edgePoints);

        // 设置外部节点下的内部空洞节点
        void SetPolygonInsidePoints(vector<Vector3> innerPoints);

        void ApplyInsidePolygonPoints();

        // 判断当前点是否为耳尖
        bool IsPointEar(PointLinkNode* checkNode);
        
        // 执行单步耳切法
        bool OneStepEarClipping();
        PointLinkNode* GetValidFirstPoint() const;

        // 执行耳切法
        vector<Triangle> EarClipping()
        {
            while(OneStepEarClipping()){}
            
            return GetGenTriangles();
        }

        // 获取所有输入的点
        vector<Vector3> GetEdgePoints()
        {
            return edgePoints;
        }

        // 获取当前未切割的点
        PointLinkNode* GetValidFirstPoint()
        {
            return firstNode;
        }

        // 获取当前所有生成的三角形
        vector<Triangle> GetGenTriangles()
        {
            return triangles;
        }
        
    private:
        // 记录原始点，逆时针顺序输入
        vector<Vector3> edgePoints;
        // 外节点链表的起点
        PointLinkNode* firstNode = nullptr;
        // 记录所有输入点中的最右点
        PointLinkNode* rightEdgeNode = nullptr;
        // 记录所有内部岛洞的头结点，为方便后续进行比较
        // 此处的first为firstNode，second为当前Node链下，最靠右的点
        vector<std::pair<PointLinkNode*, PointLinkNode*>> insideFirstNodes;
        // 分割形成的三角形集合
        vector<Triangle> triangles;
    };
}