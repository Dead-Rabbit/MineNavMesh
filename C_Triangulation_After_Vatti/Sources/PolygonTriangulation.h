#pragma once

#include <vector>

#include "../../Base/Graphes.h"
#include "../../Base/Vectors.h"
#include "../../Base/NavMath.h"
#include "../../Base/NavMeshHelper.h"

using namespace std;

namespace ZXNavMesh
{
    class PointLinkNode;
    class Triangle;
    class OutsidePolygon;

    /**
     *  耳切法类
     *  2021/10/27 版本
     *
     *  使用方式：
     *  
     *
     *  注意：
     *  1. 当前轮廓为逆时针输入，岛洞为顺时针输入
     *  2. 每一个轮廓可以拥有多个岛洞
     *  3. 岛洞与岛洞之间不可有交互！
     */
    class PolygonTriangulation
    {
    public:
        void AddPolygonPoints(vector<Vector3> points)
        {
            // 判断当前线段是否为外边框和岛洞
            if (!contourClockwise && NavMeshHelper::IsTriangleOutside(points))
                AddPolygonOutPoints(points);
            else
                AddPolygonInsidePoints(points);
        }
        
        // 设置需要进行计算的外部节点
        void AddPolygonOutPoints(vector<Vector3> edgePoints);

        // 设置外部节点下的内部空洞节点
        void AddPolygonInsidePoints(vector<Vector3> innerPoints);
        
        // 执行单步耳切法
        bool OneStepEarClipping();
        
        // 执行耳切法
        void EarClipping()
        {
            while (OneStepEarClipping()){}
        }

        vector<OutsidePolygon*> GetOutsidePolygons() const
        {
            return polygons;
        }

    private:
        bool contourClockwise = false;  // 外边框是否为顺时针；默认为逆时针
        // 记录当前所有轮廓列表
        vector<OutsidePolygon*> polygons;
        // 记录当前未被分配的岛洞列表，每个岛洞必然在一个轮廓里
        vector<vector<Vector3>> insidePolygons;
    };

    class OutsidePolygon
    {
    public:
        int num;    // 编号
        
        /** <summary></summary>
         *  <param name="edgePoints">每一个轮廓所需要的点，当前必须为逆时针输入</param>
         */
        OutsidePolygon(vector<Vector3> edgePoints);
        
        // 设置外部节点下的内部空洞节点
        void AddPolygonInsidePoints(vector<Vector3> innerPoints);
        
        // 判断当前点是否为耳尖
        bool IsPointEar(PointLinkNode* checkNode) const;
        
        // 执行单步耳切法
        bool OneStepEarClipping();
        
        // 执行耳切法
        vector<Triangle*> EarClipping()
        {
            while(OneStepEarClipping()){}
            return GetGenTriangles();
        }

        // 获取当前所有生成的三角形
        vector<Triangle*> GetGenTriangles() const
        {
            return triangles;
        }

        // 检查点是否在当前轮廓内
        bool IsPointInPolygon(Vector3 point) const;
        
    private:
        static int polygonNum;
        // 记录原始点，逆时针顺序输入
        vector<Vector3> edgePoints = vector<Vector3>();
        // 外节点链表的起点
        PointLinkNode* firstNode = nullptr;
        // 记录所有输入点中的最右点
        PointLinkNode* rightEdgeNode = nullptr;
        // 此处为firstNode，存的值为当前岛洞的最右点
        vector<PointLinkNode*> insideFirstNodes;
        // 生效所有岛洞，目前在单步耳切中有执行
        void ApplyInsidePolygonPoints();
        // 分割形成的三角形集合
        vector<Triangle*> triangles;
        // 标记是否可以开始进行剪裁了
        bool _beginEarClipping = false;
        // 重置所有内容
        void Reset()
        {
            _beginEarClipping = false;
            firstNode = nullptr;
            edgePoints.clear();
            insideFirstNodes.clear();
        }
    };
    
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

}