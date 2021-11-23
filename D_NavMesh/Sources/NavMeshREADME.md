### Ѱ··�����ɹ��ߣ���ҪΪ��Ӧ�Թ̶���Χ�ڳ����谭�������Ѱ·����

> ���̰�����ͨ����߿��谭�߿�����·��������Ѱ·����ȡ���յ�·����

```c++

// ʹ�÷���
int main(int argc, char* argv[])
{
    // 1. ʵ��������
    PolygonNavMeshTool polygonNavMeshTool = PolygonNavMeshTool();
    
    // 2. ������߿���谭�߿�
    // ������߿�
    vector<Vector3> subjectPath = vector<Vector3>();
    subjectPath.push_back(Vector3(100, 100, 0));
    subjectPath.push_back(Vector3(100, 300, 0));
    subjectPath.push_back(Vector3(300, 300, 0));
    subjectPath.push_back(Vector3(300, 100, 0));
    polygonNavMeshTool.AddPolygonOutsideContour(subjectPath);
    
    // �����谭��
    vector<Vector3> clipPath = vector<Vector3>();
    clipPath.push_back(Vector3(258, 151, 0));
    clipPath.push_back(Vector3(281, 324, 0));
    clipPath.push_back(Vector3(324, 317, 0));
    clipPath.push_back(Vector3(297, 148, 0));
    polygonNavMeshTool.AddPolygonInsideContour(clipPath);

    // 3. Ѱ������·��
    vector<Vector3> finalPathNodes = polygonNavMeshTool.FindPath(Vector3(264, 204, 0), Vector3(308, 291, 0));

    // �������
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