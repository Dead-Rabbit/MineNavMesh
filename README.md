# MineNavMesh
个人关于 寻路 的学习和实现

学习自： https://www.cnblogs.com/xignzou/p/3721494.html

---
> 在使用之前，请安装 easyx -> https://easyx.cn/download

### D_NavMesh 内包含整个流程的应用

> 流程包含：通过外边框、阻碍边框生成路径并进行寻路，获取最终的路径点

使用方法请看 D_NavMesh/Main.cpp

```c++

// 使用方法
int main(int argc, char* argv[])
{
    // 1. 实例化工具
    PolygonNavMeshTool polygonNavMeshTool = PolygonNavMeshTool();
    
    // 2. 添加外边框和阻碍边框
    // 添加外边框
    vector<Vector3> subjectPath = vector<Vector3>();
    subjectPath.push_back(Vector3(100, 100, 0));
    subjectPath.push_back(Vector3(100, 300, 0));
    subjectPath.push_back(Vector3(300, 300, 0));
    subjectPath.push_back(Vector3(300, 100, 0));
    polygonNavMeshTool.AddPolygonOutsideContour(subjectPath);
    
    // 添加阻碍框
    vector<Vector3> clipPath = vector<Vector3>();
    clipPath.push_back(Vector3(258, 151, 0));
    clipPath.push_back(Vector3(281, 324, 0));
    clipPath.push_back(Vector3(324, 317, 0));
    clipPath.push_back(Vector3(297, 148, 0));
    polygonNavMeshTool.AddPolygonInsideContour(clipPath);

    // 3. 寻找最终路径
    vector<Vector3> finalPathNodes = polygonNavMeshTool.FindPath(Vector3(264, 204, 0), Vector3(308, 291, 0)s);

    // 4. 输出测试
    if (finalPathNodes.size() > 0)
    {
        for (int z = 0; z < finalPathNodes.size() - 1; z++)
        {
            const auto point = finalPathNodes[z];
            const auto nextPoint = finalPathNodes[z + 1];
            std::cout << point << " -> " << nextPoint << std::endl;
        }
    }
}

```

### A_TriangulationByEarClipping
---
使用耳切法切分多边形

使用方法可见 A_TriangulationByEarClipping/Main.cpp


```c++
    triangulationTool = PolygonTriangulation();
    
    // 定义多边形的点，介于 EasyX 使用的是整型，此处尽量使用整型来测试
    vector<Vector3> edgePoints;
    edgePoints.push_back(Vector3(43, 257, 0));
    edgePoints.push_back(Vector3(160, 391, 0));
    edgePoints.push_back(Vector3(378, 199, 0));
    edgePoints.push_back(Vector3(520, 358, 0));
    edgePoints.push_back(Vector3(602, 234, 0));
    edgePoints.push_back(Vector3(482, 223, 0));
    edgePoints.push_back(Vector3(383, 101, 0));
    edgePoints.push_back(Vector3(242, 248, 0));
    edgePoints.push_back(Vector3(116, 249, 0));
    edgePoints.push_back(Vector3(128, 99, 0));

    triangulationTool.SetPolygonPoints(edgePoints);
    
    // 获取切分后的三角形
    vector<Triangle> triangles = triangulationTool.EarClipping();
```
