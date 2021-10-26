/**
 * 学习自： https://www.cnblogs.com/xignzou/p/3721494.html
 *
 * 备注：输出文字的用法
 * setbkmode(TRANSPARENT);     // 去掉文字背景颜色
 * settextstyle(20, 0, L"微软雅黑");
 * setcolor(BLACK);
 * TCHAR str[25];
 * _stprintf_s(str, _T("%d(%.1f,%.1f)"), outCurNode->num, point.x, point.y);
 * outtextxy(point.x - 5, point.y - 20, str);
*/

#define USE_EASYX_GTAPHICS  // 是否使用 EasyX 进行输出，目前EasyX仅支持Windows平台
#define DEBUG_STEP   // 是否分步骤调试，前提是开启了 USE_EASYX_GRAPHICS

#include <vector>

#include "../Base/Vectors.h"
#include "Sources/PolygonTriangulation.h"

#ifdef USE_EASYX_GTAPHICS
#include <graphics.h>   // 如果此处出现了错误，请安装 EasyX 库，如果不想安装，注释掉13行的 #define USE_EASYX_GTAPHICS 即可
#include <conio.h>      //为了使用_getch()
#endif

using namespace std;
using namespace ZXNavMesh;

#ifdef USE_EASYX_GTAPHICS

// 屏幕大小
Vector2* graphSize = new Vector2(640, 480);

// 重新绘制
void DrawClippingBoard();

bool ifFinishClipping = false;

#endif

PolygonTriangulation triangulationTool;

int main()
{
    // 使用耳切法三角化工具
    triangulationTool = PolygonTriangulation();
    
    // 定义多边形的点，介于 EasyX 使用的是整型，此处尽量使用整型来测试
    vector<Vector3> edgePoints;
    edgePoints.clear();
    edgePoints.push_back(Vector3(43, 257, 0));
    edgePoints.push_back(Vector3(160, 391, 0));
    edgePoints.push_back(Vector3(349, 380, 0));
    edgePoints.push_back(Vector3(520, 358, 0));
    edgePoints.push_back(Vector3(602, 234, 0));
    edgePoints.push_back(Vector3(482, 223, 0));
    edgePoints.push_back(Vector3(383, 101, 0));
    edgePoints.push_back(Vector3(227, 192, 0));
    edgePoints.push_back(Vector3(116, 249, 0));
    edgePoints.push_back(Vector3(128, 99, 0));
    triangulationTool.SetPolygonOutPoints(edgePoints);

    // 测试新增岛洞输入
    vector<Vector3> insidePoints;
    
    insidePoints.clear();
    insidePoints.push_back(Vector3(304, 219, 0));
    insidePoints.push_back(Vector3(345, 234, 0));
    insidePoints.push_back(Vector3(261, 307, 0));
    insidePoints.push_back(Vector3(312, 355, 0));
    insidePoints.push_back(Vector3(223, 364, 0));
    insidePoints.push_back(Vector3(153, 337, 0));
    insidePoints.push_back(Vector3(173, 273, 0));
    insidePoints.push_back(Vector3(235, 218, 0));
    triangulationTool.AddPolygonInsidePoints(insidePoints);
    
    insidePoints.clear();
    insidePoints.push_back(Vector3(416, 216, 0));
    insidePoints.push_back(Vector3(421, 272, 0));
    insidePoints.push_back(Vector3(362, 263, 0));
    insidePoints.push_back(Vector3(375, 220, 0));
    triangulationTool.AddPolygonInsidePoints(insidePoints);
    
    insidePoints.clear();
    insidePoints.push_back(Vector3(335, 312, 0));
    insidePoints.push_back(Vector3(301, 329, 0));
    insidePoints.push_back(Vector3(285, 302, 0));
    insidePoints.push_back(Vector3(311, 281, 0));
    triangulationTool.AddPolygonInsidePoints(insidePoints);
    
    insidePoints.clear();
    insidePoints.push_back(Vector3(419, 301, 0));
    insidePoints.push_back(Vector3(439, 264, 0));
    insidePoints.push_back(Vector3(428, 218, 0));
    insidePoints.push_back(Vector3(429, 191, 0));
    insidePoints.push_back(Vector3(498, 244, 0));
    triangulationTool.AddPolygonInsidePoints(insidePoints);

#ifdef USE_EASYX_GTAPHICS
    
    initgraph(graphSize->x, graphSize->y);    // 创建绘图窗口，大小为 640x480 像素
    setbkmode(TRANSPARENT);     // 去掉文字背景颜色

#ifdef DEBUG_STEP
    // 此处仅为调试用，不建议在别的地方手动执行
    triangulationTool.ApplyInsidePolygonPoints();
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
            ifFinishClipping = !triangulationTool.OneStepEarClipping();
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
    settextstyle(20, 0, L"微软雅黑");
    
    auto firstNode = triangulationTool.GetValidFirstPoint();
    PointLinkNode* curNode = firstNode;
    if (curNode == nullptr)
        return;
    
    do
    {
        const Vector3 point = curNode->point;
        const Vector3 nextPoint = curNode->nextNode->point;
            
        // 绘制点
        setlinecolor(BLACK);
        setfillcolor(RED);
        
        fillcircle(point.x, point.y, 3);

        // 绘制点信息
        setcolor(BLACK);
        TCHAR str[25];
        if (curNode == firstNode)
            _stprintf_s(str, _T("*%d(%.1f,%.1f)"), curNode->num, point.x, point.y);
        else
            _stprintf_s(str, _T("%d(%.1f,%.1f)"), curNode->num, point.x, point.y);
        outtextxy(point.x - 5, point.y - 20, str);
        
        // 绘制线
        setlinecolor(BLACK);
        line(point.x, point.y, nextPoint.x, nextPoint.y);
        curNode = curNode->nextNode;
    } while(curNode != firstNode);
}

void DrawTriangles()
{
    const auto triangles = triangulationTool.GetGenTriangles();
    if (triangles.size() == 0)
        return;
    
    setfillcolor(0xF0FFF0);
    setlinecolor(GREEN);
    for (const Triangle triangle : triangles)
    {
        line(triangle.A.x, triangle.A.y, triangle.B.x, triangle.B.y);
        line(triangle.B.x, triangle.B.y, triangle.C.x, triangle.C.y);
        line(triangle.C.x, triangle.C.y, triangle.A.x, triangle.A.y);
        int points[] = {triangle.A.x, triangle.A.y, triangle.B.x, triangle.B.y, triangle.C.x, triangle.C.y};
        fillpoly(3, points);
    }
}

// 绘制提示内容
void DrawOptionHelper()
{
    settextstyle(25, 0, L"微软雅黑");
    setcolor(BLACK);
    TCHAR str[25];
#ifdef DEBUG_STEP
    if (!ifFinishClipping)
        _stprintf_s(str, _T("点击左键继续"));
    else
#endif
        _stprintf_s(str, _T("当前已完成"));
    
    outtextxy(0, 0, str);
}

void DrawClippingBoard()
{
    // 清屏
    ClearDrawBoard();

    // 绘制提示内容
    DrawOptionHelper();
    
    // 绘制三角形
    DrawTriangles();
    
    // 绘制点和线
    DrawPointAndLine();
}

#endif
