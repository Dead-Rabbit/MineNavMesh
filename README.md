# MineNavMesh
个人关于 寻路 的学习和实现

学习自： https://www.cnblogs.com/xignzou/p/3721494.html

---
> 在使用之前，请安装 easyx -> https://easyx.cn/download

---
### TriangulationByEarClipping
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