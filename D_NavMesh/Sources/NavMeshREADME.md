### 寻路路径生成工具，主要为了应对固定范围内出现阻碍的情况的寻路处理

> 流程包含：通过外边框、阻碍边框生成路径并进行寻路，获取最终的路径点

```c++

// 使用方法
int main(int argc, char* argv[])
{
    // 1. 实例化工具
    PolygonNavMeshTool polygonNavMeshTool = PolygonNavMeshTool();
    
    // 2. 添加外边框和阻碍边框
    // 添加外边框
    vector<NavMeshBase::Vector3> subjectPath = vector<NavMeshBase::Vector3>();
    subjectPath.push_back(NavMeshBase::Vector3(100, 100, 0));
    subjectPath.push_back(NavMeshBase::Vector3(100, 300, 0));
    subjectPath.push_back(NavMeshBase::Vector3(300, 300, 0));
    subjectPath.push_back(NavMeshBase::Vector3(300, 100, 0));
    polygonNavMeshTool.AddPolygonOutsideContour(subjectPath);
    
    // 添加阻碍框
    vector<NavMeshBase::Vector3> clipPath = vector<NavMeshBase::Vector3>();
    clipPath.push_back(NavMeshBase::Vector3(258, 151, 0));
    clipPath.push_back(NavMeshBase::Vector3(281, 324, 0));
    clipPath.push_back(NavMeshBase::Vector3(324, 317, 0));
    clipPath.push_back(NavMeshBase::Vector3(297, 148, 0));
    polygonNavMeshTool.AddPolygonInsideContour(clipPath);

    // 3. 寻找最终路径
    vector<NavMeshBase::Vector3> finalPathNodes = polygonNavMeshTool.FindPath(NavMeshBase::Vector3(264, 204, 0), NavMeshBase::Vector3(308, 291, 0));

    // 输出测试
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
