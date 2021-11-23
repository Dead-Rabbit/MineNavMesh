#pragma once
#include<string>

#include "PolygonTriangulation.h"
using namespace std;

namespace PolygonNavMesh
{
    /*
    本程序是使用Dijkstra算法实现求解最短路径的问题
    采用的邻接矩阵来存储图
    */
    //记录起点到每个顶点的最短路径的信息
    struct Dis {
        // string path;
        vector<ClipTriangle*> pathTriangle;
        int value;
        bool visit;
        Dis() {
            visit = false;
            value = 0;
            // path = "";
        }
    };

    class Graph_DG {
    private:
        ClipTriangle* startClipTriangle;
        int vexnum;                 //图的顶点个数
        int **arc;                  //邻接矩阵
        ClipTriangle* *triangles_;  // 输入记录的所有三角形
        Dis * dis;                  //记录各个顶点最短路径的信息
    public:
        Graph_DG(){};
        ~Graph_DG();
        
        //创建图
        void createGraph(vector<ClipTriangle*> inputTriangles);
        // 输入起始、结束点进行搜索 所有路过的三角形
        vector<ClipTriangle*> find_path_triangles(const ClipTriangle* endTriangle) const;
        
        //求最短路径
        void Dijkstra(ClipTriangle* beginTriangle);
        
        //打印最短路径
        void print_path(int);
    };

}