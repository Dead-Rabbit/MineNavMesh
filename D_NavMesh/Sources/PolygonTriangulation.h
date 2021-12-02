#pragma once

#include "../../Base/Vectors.h"
#include "../../Base/NavMath.h"
#include "../../Base/NavMeshHelper.h"

/**
 *  本类辅助于 PolygonNavMeshTool 用于针对轮廓进行三角化
 */
namespace PolygonNavMesh
{
    class PointLinkNode;
    class ClipLine;
    class ClipTriangle;
    class OutsidePolygon;

    /**
     *  耳切法类
     */
    class PolygonTriangulation
    {
    public:
        void AddPolygonPoints(std::vector<NavMeshBase::Vector3> points)
        {
            // 判断当前线段是否为外边框和岛洞
            if (!contourClockwise && NavMeshBase::NavMeshHelper::IsTriangleOutside(points))
                AddPolygonOutPoints(points);
            else
                AddPolygonInsidePoints(points);
        }
        
        // 设置需要进行计算的外部节点
        void AddPolygonOutPoints(std::vector<NavMeshBase::Vector3> edgePoints);

        // 设置外部节点下的内部空洞节点
        void AddPolygonInsidePoints(std::vector<NavMeshBase::Vector3> innerPoints);
        
        // 执行单步耳切法
        bool OneStepEarClipping();
        
        // 执行耳切法
        void EarClipping()
        {
            while (OneStepEarClipping()){}
        }

        // 获取所有外边框形状
        std::vector<OutsidePolygon*> GetOutsidePolygons() const
        {
            return polygons;
        }

        // 获取所有生成三角形列表
        std::vector<std::vector<ClipTriangle*>> GetGenTriangles() const;

        void Reset()
        {
            polygons.clear();
            insidePolygons.clear();
        }

    private:
        bool contourClockwise = false;  // 外边框是否为顺时针；默认为逆时针
        // 记录当前所有轮廓列表
        std::vector<OutsidePolygon*> polygons;
        // 记录当前未被分配的岛洞列表，每个岛洞必然在一个轮廓里
        std::vector<std::vector<NavMeshBase::Vector3>> insidePolygons;
    };

    class OutsidePolygon
    {
    public:
        int num;    // 编号
        
        /** <summary></summary>
         *  <param name="edgePoints">每一个轮廓所需要的点，当前必须为逆时针输入</param>
         */
        OutsidePolygon(std::vector<NavMeshBase::Vector3> edgePoints);
        
        // 设置外部节点下的内部空洞节点
        void AddPolygonInsidePoints(std::vector<NavMeshBase::Vector3> innerPoints);
        
        // 判断当前点是否为耳尖
        bool IsPointEar(PointLinkNode* checkNode) const;
        
        // 执行单步耳切法
        bool OneStepEarClipping();
        
        // 执行耳切法
        std::vector<ClipTriangle*> EarClipping()
        {
            while(OneStepEarClipping()){}
            return GetGenTriangles();
        }

        // 获取当前所有生成的三角形
        std::vector<ClipTriangle*> GetGenTriangles() const
        {
            return triangles;
        }

        PointLinkNode* GetFirstNode() const
        {
            return firstNode;
        }

        std::vector<PointLinkNode*> GetInsideFirstNodes() const
        {
            return insideFirstNodes;
        }

        // 检查点是否在当前轮廓内
        bool IsPointInPolygon(NavMeshBase::Vector3 point) const;
        
        // 获取轮廓外部点到当前轮廓最近的点，返回两个对应的 LinkPoint
        bool GetNearCrossFromOutsidePoint(NavMeshBase::Vector3 point, NavMeshBase::Vector3& nearPoint);
        
        // 获取轮廓内部点到当前轮廓中岛洞最近的点，返回两个对应的 LinkPoint
        bool GetNearCrossFromInsidePoint(NavMeshBase::Vector3 point, NavMeshBase::Vector3& nearPoint);

        // 获取轮廓点
        std::vector<NavMeshBase::Vector3> GetEdgePoints() const {return edgePoints;}
        
    private:
        static int polygonNum;
        // 记录原始点，逆时针顺序输入
        std::vector<NavMeshBase::Vector3> edgePoints = std::vector<NavMeshBase::Vector3>();
        // 外节点链表的起点
        PointLinkNode* firstNode = nullptr;
        // 记录所有输入点中的最右点
        PointLinkNode* rightEdgeNode = nullptr;
        // 此处为firstNode，存的值为当前岛洞的最右点
        std::vector<PointLinkNode*> insideFirstNodes = std::vector<PointLinkNode*>();
        std::vector<std::vector<NavMeshBase::Vector3>> insidePointsList = std::vector<std::vector<NavMeshBase::Vector3>>();
        // 生效所有岛洞，目前在单步耳切中有执行
        void ApplyInsidePolygonPoints();
        // 分割形成的三角形集合
        std::vector<ClipTriangle*> triangles;
        // 标记是否可以开始进行剪裁了
        bool _beginEarClipping = false;
        // 重置所有内容
        void Reset()
        {
            _beginEarClipping = false;
            firstNode = nullptr;
            edgePoints.clear();
            insideFirstNodes.clear();
            insidePointsList.clear();
        }
    };

    /**
     * 耳切法中会使用的线
     * 方向为 A -> B
     * Line的方向来自 Polygon 中的逆时针路线
     */
    class ClipLine
    {
    public:
        PointLinkNode* A = nullptr;
        PointLinkNode* B = nullptr;
    };

    /**
     *  耳切法最终生成的三角形
     *  顶点的存放数据为 (A->C->B->A 逆时针)
     */
    class ClipTriangle
    {
    public:
        ClipTriangle(PointLinkNode* A, PointLinkNode* B, PointLinkNode* C, int numInPolygon);

        PointLinkNode* A = nullptr;
        PointLinkNode* B = nullptr;
        PointLinkNode* C = nullptr;

        std::vector<PointLinkNode*> points;  // 记录A B C；方便后续遍历用

        int num;
        int numInPolygon = 0;   // 当前三角形在一个轮廓中的编号，从0开始，方便后面使用Dijkstra搜索
        NavMeshBase::Vector3 centerPos;

        // 获取相连通的其他三角形
        std::vector<std::pair<ClipTriangle*, ClipLine*>> GetLinkedClipTriangles();

        /**
         *  <summary>检查点是否为当前的轮廓点，包括轮廓点的影子点</summary>
         *  <param name="otherTriangle">检查的其他三角形</param>
         *  <param name="outputLine">裁剪出的线</param>
         */
        bool IsTriangleLink(const ClipTriangle* otherTriangle, ClipLine* outputLine);

        /**
         *  <summary>检查点是否在当前三角形内</summary>
         *  <param name="point">其他点</param>
         *  <param name="diff">误差</param>
         */
        bool IsPointInTriangle(NavMeshBase::Vector3 point, double diff = 0);
        
    private:
        static int triangleNum;
        bool InitLinkedTriangle = false;
        std::vector<std::pair<ClipTriangle*, ClipLine*>> linkedTriangles;
        // 获取相连通的其他三角形
        void GetLinkedClipTrianglesByPoint(PointLinkNode* point);
    };
    
    // 耳切法使用点链表节点
    class PointLinkNode
    {
    public:
        PointLinkNode(double x, double y, double z)
        {
            this->point = NavMeshBase::Vector3(x, y, z);
            this->num = ++pointNum;
        }

        // 当前节点编号，方便调试
        int num;
	
        // 当前点
        NavMeshBase::Vector3 point;
        
        // 双向链 - 动态，不可用于后续判断
        PointLinkNode* preNode = nullptr;
        PointLinkNode* nextNode = nullptr;

        // 影子节点 - 从当前点拷贝或拷贝自其他点，表示连通;LinkNode 只可能有一个
        PointLinkNode* linkNode = nullptr;
        
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
            return NavMeshBase::NavMath::Cross(
                NavMeshBase::Vector3(point - preNode->point), NavMeshBase::Vector3(point - nextNode->point)
            ).z > 0;
        };

        // 相连接的三角形
        std::vector<ClipTriangle*> linkTriangles;
        
        // 将三角形插入到当前点的链接记录中
        void AddLinkTriangle(ClipTriangle* triangle)
        {
            linkTriangles.push_back(triangle);
        }

        // 获取所有点相连接的三角形，包括影子点的链接三角形
        std::vector<ClipTriangle*> GetLinkTriangles()
        {
            std::vector<ClipTriangle*> result;
            if (linkNode != nullptr)
            {
                auto linkNodeTriangles = linkNode->linkTriangles;
                result.insert(result.end(), linkNodeTriangles.begin(), linkNodeTriangles.end());
            }
            result.insert(result.end(), linkTriangles.begin(), linkTriangles.end());
            return result;
        }
        
    private:
        static int pointNum;
    };

}