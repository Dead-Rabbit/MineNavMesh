/**
 * 学习自： https://www.cnblogs.com/xignzou/p/3721494.html
*/

// #define USE_EASYX_GTAPHICS  // 是否使用 EasyX 进行输出，目前EasyX仅支持Windows平台
// #define DEBUG_STEP   // 是否分步骤调试，前提是开启了 USE_EASYX_GRAPHICS

#include <vector>

#include "../Base/Vectors.h"
#include "Sources/PolygonTriangulation.h"

#ifdef USE_EASYX_GTAPHICS
#include <graphics.h>
#include <conio.h>      //为了使用_getch()
#endif

using namespace std;
using namespace ZXNavMesh;

#ifdef USE_EASYX_GTAPHICS
// 屏幕大小
Vector2* graphSize = new Vector2(640, 480);

// 重新绘制
void DrawClippingBoard();

#endif

PolygonTriangulation triangulationTool;

int main()
{
    // 使用耳切法三角化工具
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
    
#ifdef USE_EASYX_GTAPHICS
    
    initgraph(graphSize->x, graphSize->y);    // 创建绘图窗口，大小为 640x480 像素
    setbkmode(TRANSPARENT);     // 去掉文字背景颜色

#ifdef DEBUG_STEP
    // 绘制内容
    DrawClippingBoard();
    
    ExMessage m;		// Define a message variable
    while(true)
    {
        // Get a mouse message or a key message
        m = getmessage(EM_MOUSE | EM_KEY);
        switch(m.message)
        {
        case WM_LBUTTONDOWN:
            if (triangulationTool.OneStepEarClipping())
                std::cout << "成功" << endl;
            else
                std::cout << "失败" << endl;

            DrawClippingBoard();
            break;
        case WM_RBUTTONDOWN:
            std::cout << "(" << m.x << ", " << m.y << ")" << endl;
            break;
        case WM_KEYDOWN:
            if (m.vkcode == VK_ESCAPE)
                return 0;	// Press ESC key to exit
        default: break;
        }
    }
#else
    triangulationTool.EarClipping();
#endif
    // 绘制分割的三角形
    DrawClippingBoard();
    // 无输出，多次执行分割直至完毕
    _getch();
    closegraph();          // 关闭绘图窗口
#else
    // 当前不使用 EasyX 进行绘制
    
    triangulationTool.EarClipping();
    // 输出分割三角形内容
    for (Triangle triangle : triangulationTool.GetGenTriangles())
    {
        std::cout << "三角形：A(" << triangle.A.x << ", " << triangle.A.y
            << ")-B(" << triangle.B.x << ", " << triangle.B.y
            << ")-C(" << triangle.C.x << ", " << triangle.C.y << ")" << endl;
    }
#endif
    
    return 0;
}

#ifdef USE_EASYX_GTAPHICS

void ClearDrawBoard()
{
    cleardevice();
    setfillcolor(WHITE);
    solidrectangle(0, 0, graphSize->x, graphSize->y); // 填充背景色
}

// 使用EasyX 输出点和线
void DrawPointAndLine()
{
    vector<Vector3> points = triangulationTool.GetValidPoints();
    for (int i = 0; i < points.size(); i++)
    {
        const Vector3 point = points[i];
        const Vector3 nextPoint = points[i == points.size() - 1 ? 0 : i + 1];
            
        // 绘制点
        setlinecolor(BLACK);
        setfillcolor(RED);
        
        fillcircle(point.x, point.y, 3);

        // 绘制线
        setlinecolor(BLACK);
        line(point.x, point.y, nextPoint.x, nextPoint.y);
    }
}

void DrawTriangles()
{
    const auto triangles = triangulationTool.GetGenTriangles();
    if (triangles.size() == 0)
        return;
    
    setlinecolor(GREEN);
    for (const Triangle triangle : triangles)
    {
        line(triangle.A.x, triangle.A.y, triangle.B.x, triangle.B.y);
        line(triangle.B.x, triangle.B.y, triangle.C.x, triangle.C.y);
        line(triangle.C.x, triangle.C.y, triangle.A.x, triangle.A.y);
    }
}

void DrawClippingBoard()
{
    // 清屏
    ClearDrawBoard();
    // 绘制点和线
    DrawPointAndLine();
    // 绘制三角形
    DrawTriangles();
}

#endif
