#pragma once

#include <vector>

#include "PolygonTriangulation.h"
#include "PolygonTriangulation.h"
#include "PolygonTriangulation.h"
#include "PolygonTriangulation.h"
#include "../../Base/Vectors.h"
#include "../../Base/NavMath.h"
#include "../../Base/NavMeshHelper.h"

using namespace std;

namespace ZXNavMesh
{
    class PointLinkNode;
    class ClipLine;
    class ClipTriangle;
    class OutsidePolygon;

    /**
     *  耳切法类
     *  2021/10/27 版本
     *
     *  使用方式：
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
        vector<ClipTriangle*> EarClipping()
        {
            while(OneStepEarClipping()){}
            return GetGenTriangles();
        }

        // 获取当前所有生成的三角形
        vector<ClipTriangle*> GetGenTriangles() const
        {
            return triangles;
        }

        PointLinkNode* GetFirstNode() const
        {
            return firstNode;
        }

        vector<PointLinkNode*> GetInsideFirstNodes() const
        {
            return insideFirstNodes;
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
        vector<ClipTriangle*> triangles;
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

    /**
     *  裁剪线
     */
    class ClipLine
    {
    public:
        inline ClipLine(PointLinkNode* start, PointLinkNode* end)
        {
            A = start;
            B = end;
            num = ++lineNum;
        }

        int num;
		
        PointLinkNode* A;
        PointLinkNode* B;

        // 一个线连接的两个三角形，在裁剪的时候生成
        vector<ClipTriangle*> triangles;
    private:
        static int lineNum;
    };
    
    /**
     *  耳切法最终生成的三角形
     */
    class ClipTriangle
    {
    public:
        ClipTriangle(PointLinkNode* A, PointLinkNode* B, PointLinkNode* C);

        PointLinkNode* A = nullptr;
        PointLinkNode* B = nullptr;
        PointLinkNode* C = nullptr;

        int num;
        Vector3 centerPos;

        // 三角形对应三个线
        // vector<ClipLine*> lines;
    private:
        static int triangleNum;
        // ClipLine* CreateTriangleLine(PointLinkNode* pA, PointLinkNode* pB);
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

        // 当前节点编号，方便调试
        int num;
	
        // 当前点
        Vector3 point;
        
        // 双向链
        PointLinkNode* preNode = nullptr;
        PointLinkNode* nextNode = nullptr;

        PointLinkNode* CopyNode()
        {
            PointLinkNode* otherNode = new PointLinkNode(point.x, point.y, point.z);
            linkNode = otherNode;
            otherNode->linkNode = this;
            return otherNode;
        }

        // 计算当前角是否为凸角
        bool IsPointConvex() const
        {
            return NavMath::Cross(
                Vector3(point - preNode->point), Vector3(point - nextNode->point)
            ).z > 0;
        };

        // // 根据另外一个点获取当前点对应的线段
        // ClipLine* GetLineByOtherNode(PointLinkNode* otherNode)
        // {
        //     for (int i = 0; i < lines.size(); ++i)
        //     {
        //         ClipLine* line = lines[i];
        //         if (line->B == otherNode || line->A == otherNode)
        //             return line;
        //     }
        //
        //     return nullptr;
        // }

        // void AddLine(ClipLine* newLine)
        // {
        //     lines.push_back(newLine);
        // }
        
    private:
        static int pointNum;
        
        // 影子节点 - 从当前点拷贝或拷贝自其他点，表示连通
        PointLinkNode* linkNode = nullptr;

        // 点的所有连线，在三角形裁剪的时候生成
        // vector<ClipLine*> lines;
    };

}