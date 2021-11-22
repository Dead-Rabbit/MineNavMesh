﻿#pragma once
#include <list>
#include <map>

#include "clipper.h"
#include "PolygonTriangulation.h"

using namespace clipperlib;

namespace PolygonNavMesh
{
    /**
     *  多边形寻路工具
     *  使用方法: 详情查看 ./NavMeshREADME.md
     */
    class PolygonNavMeshTool
    {
    public:
        PolygonNavMeshTool()
        {
            triangulationTool = PolygonTriangulation();
        }

        /**
         *  <summary>追加外边轮廓，轮廓按照逆时针输入</summary>
         *  <param name="contour">轮廓</param>
         */
        void AddPolygonOutsideContour(vector<Vector3> contour);
        
        /**
         *  <summary>追加岛洞轮廓，轮廓按照逆时针输入</summary>
         *  <param name="contour">轮廓</param>
         *  <returns>内边框的编号</returns>
         */
        int AddPolygonInsideContour(vector<Vector3> contour);

        /**
         *  <summary>删除追加的岛洞轮廓</summary>
         *  <param name="contourIndex">岛洞轮廓的编号</param>
         *  <returns>是否成功删除</returns>
         */
        bool RemovePolygonInsideContour(int contourIndex);

        // 进行 vatti 切割 + 三角化
        vector<vector<ClipTriangle*>> GenerateFinalTriangles();

        /**
         *  <summary>输入起始点，获取路径点</summary>
         *  <param name="start">起始点</param>
         *  <param name="end">结束点</param>
         */
        vector<Vector3> FindPath(Vector3 start, Vector3 end);

        vector<OutsidePolygon*> GetOutsidePolygons() const
        {
            return triangulationTool.GetOutsidePolygons();
        }

        // 获取当前生成的所有三角形
        vector<vector<ClipTriangle*>> GetGenTriangles() const
        {
            return triangulationTool.GetGenTriangles();
        }
        
    private:
        bool genTriangleDirty = false;
        PolygonTriangulation triangulationTool; // 耳切法三角化工具

        // 记录的轮廓
        vector<Path<double>> subjectPaths;
        map<size_t, Path<double>> clipPathMap;
        vector<size_t> _RemovedClipPathIndex;
    };
    
}
