#include"Dijkstra.h"

namespace PolygonNavMesh
{
    //析构函数
    Graph_DG::~Graph_DG() {
        delete[] dis;
        for (int i = 0; i < this->vexnum; i++) {
            delete this->arc[i];
        }
        delete arc;
    }

    void Graph_DG::createGraph(vector<ClipTriangle*> inputTriangles) {
        //初始化顶点数
        this->vexnum = inputTriangles.size();
        //为邻接矩阵开辟空间和赋初值
        arc = new int*[this->vexnum];
        triangles_ = new ClipTriangle*[this->vexnum];
        dis = new Dis[this->vexnum];
        for (int i = 0; i < this->vexnum; i++) {
            arc[i] = new int[this->vexnum];
            for (int k = 0; k < this->vexnum; k++) {
                //邻接矩阵初始化为无穷大
                arc[i][k] = INT_MAX;
            }
        }
        
        // 计算每个节点彼此之间的权重
        int count = 0;
        for (ClipTriangle* triangle : inputTriangles)
        {
            triangles_[triangle->numInPolygon] = triangle;
            // 拿到当前三角形的所有相邻三角形
            auto linkedTriangles = triangle->GetLinkedClipTriangles();
            for (const auto linkedTrianglePair : linkedTriangles)
            {
                const ClipTriangle* linkedTriangle = linkedTrianglePair.first;
                const int weight = (triangle->centerPos - linkedTriangle->centerPos).squaredLength();
                //对邻接矩阵对应上的点赋值
                arc[triangle->numInPolygon][linkedTriangle->numInPolygon] = weight;
                //无向图添加上这行代码
                arc[linkedTriangle->numInPolygon][triangle->numInPolygon] = weight;
                ++count;
            }
        }
    }

    void Graph_DG::Dijkstra(ClipTriangle* beginTriangle){
        startClipTriangle = beginTriangle;
        const int begin = beginTriangle->numInPolygon;
        //首先初始化我们的dis数组
        int i;
        for (i = 0; i < this->vexnum; i++) {
            //设置当前的路径
            dis[i].path = "v" + to_string(begin) + "-->v" + to_string(i);
            dis[i].value = arc[begin][i];
            dis[i].pathTriangle.push_back(triangles_[i]);
        }
    
        //设置起点的到起点的路径为0
        dis[begin].value = 0;
        dis[begin].visit = true;

        int count = 1;
        
        //计算剩余的顶点的最短路径
        while (count != this->vexnum) {
            //temp用于保存当前dis数组中最小的那个下标
            //min记录的当前的最小值
            int temp=0;
            int min = INT_MAX;
            for (i = 0; i < this->vexnum; i++) {
                if (!dis[i].visit && dis[i].value < min) {
                    min = dis[i].value;
                    temp = i;
                }
            }
            //把temp对应的顶点加入到已经找到的最短路径的集合中
            dis[temp].visit = true;
            ++count;
            for (i = 0; i < this->vexnum; i++) {
                //注意这里的条件arc[temp][i]!=INT_MAX必须加，不然会出现溢出，从而造成程序异常
                if (!dis[i].visit && arc[temp][i]!=INT_MAX && (dis[temp].value + arc[temp][i]) < dis[i].value) {
                    //如果新得到的边可以影响其他为访问的顶点，那就就更新它的最短路径和长度
                    dis[i].value = dis[temp].value + arc[temp][i];
                    dis[i].path = dis[temp].path + "-->v" + to_string(i);
                    
                    dis[i].pathTriangle.clear();
                    dis[i].pathTriangle.insert(dis[i].pathTriangle.begin(), dis[temp].pathTriangle.begin(), dis[temp].pathTriangle.end());
                    dis[i].pathTriangle.push_back(triangles_[i]);
                }
            }
        }
    }
    
    vector<ClipTriangle*> Graph_DG::find_path_triangles(const ClipTriangle* endTriangle) const
    {
        vector<ClipTriangle*> pathTriangles;
        pathTriangles.push_back(startClipTriangle);
        for (ClipTriangle* pathTriangle : dis[endTriangle->numInPolygon].pathTriangle)
        {
            pathTriangles.push_back(pathTriangle);
        }
        
        return pathTriangles;
    }
}