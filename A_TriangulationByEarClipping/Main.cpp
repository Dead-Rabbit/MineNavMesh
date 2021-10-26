/**
 * 学习自： https://www.cnblogs.com/xignzou/p/3721494.html
*/

#define USE_EASYX_GTAPHICS  // 是否使用 EasyX 进行输出，目前EasyX仅支持Windows平台
#define DEBUG_STEP   // 是否分步骤调试，前提是开启了 USE_EASYX_GRAPHICS

#include <vector>

#include "../Base/NavMath.h"
#include "../Base/Vectors.h"
#include "Sources/PointLinkNode.h"

#ifdef USE_EASYX_GTAPHICS
#include <graphics.h>
#include <conio.h>      //为了使用_getch()
#endif

using namespace std;
using namespace ZXNavMesh;

// 记录原始点，逆时针顺序输入
std::vector<Vector3> edgePoints;
// 链表的起点
PointLinkNode* firstNode = nullptr;
// 分割形成的三角形集合
std::vector<Triangle> triangles;

#ifdef USE_EASYX_GTAPHICS
// 屏幕大小
Vector2* graphSize = new Vector2(640, 480);

// 重新绘制
void DrawClippingBoard();

#endif

// 判断当前点是否为耳尖
bool IsPointEar(PointLinkNode* checkNode);

// 单步耳切法
bool EarClipping();

int main()
{
    // 定义多边形的点，介于 EasyX 使用的是整型，此处尽量使用整型来测试
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
    const int pointSize = edgePoints.size();

    // 生成链表
    PointLinkNode* curNode = firstNode;
    PointLinkNode* preNode = firstNode;
    for (int i = 0; i < pointSize; i++)
    {
        if (firstNode == nullptr)
        {
            firstNode = new PointLinkNode(i+1, edgePoints[i]);
            curNode = firstNode;
            preNode = curNode;
            continue;
        }
        
        curNode->nextNode = new PointLinkNode(i+1, edgePoints[i]);
        preNode = curNode;
        curNode = curNode->nextNode;
        curNode->preNode = preNode;
    }
    curNode->nextNode = firstNode;
    firstNode->preNode = curNode;
    
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
            if (EarClipping())
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
    while(EarClipping()){}
#endif
    
    // 绘制分割的三角形
    DrawClippingBoard();
    // 无输出，多次执行分割直至完毕
    _getch();
    closegraph();          // 关闭绘图窗口
#else
    // 当前不使用 EasyX 进行绘制
    while(EarClipping()){}
    // 输出分割三角形内容
    for (Triangle triangle : triangles)
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
    PointLinkNode* outCurNode = firstNode;
    if (outCurNode != nullptr)
        do {
            // std::cout<< "点" << outCurNode->num << "->" << outCurNode->nextNode->num << endl;
            // 绘制点
            const auto point = outCurNode->point;
            setlinecolor(BLACK);
            
            // 红色代表为凹角
            // 黄色代表为凸角，但不是耳尖
            // 绿色代表耳尖
            if (IsPointEar(outCurNode))
            {
                setfillcolor(GREEN);
            }
            else
                if (outCurNode->IsPointConvex())
                    setfillcolor(YELLOW);
                else
                    setfillcolor(RED);
            
            fillcircle(point.x, point.y, 3);

            // 绘制信息显示
            setcolor(BLACK);
            settextstyle(20, 0, L"微软雅黑");
            TCHAR str[25];
            _stprintf_s(str, _T("%d(%.1f,%.1f)"), outCurNode->num, point.x, point.y);
            outtextxy(point.x - 5, point.y - 20, str);
            
            // 绘制线
            setlinecolor(BLACK);
            const auto nextPoint = outCurNode->nextNode->point;
            line(point.x, point.y, nextPoint.x, nextPoint.y);
            outCurNode = outCurNode->nextNode;
        } while(outCurNode != firstNode);
}

void DrawTriangles()
{
    if (triangles.size() == 0)
        return;

    setlinecolor(GREEN);
    for (Triangle triangle : triangles)
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

// 判断这个点是否为耳尖
bool IsPointEar(PointLinkNode* checkNode)
{
    // 首先排除当前不是一个凸边的情况
    if (!checkNode->IsPointConvex())
        return false;
    
    // 检查当前三角形内是否包含其他点
    PointLinkNode* curNode = firstNode;
    if (curNode == nullptr)
        return false;
    
    do
    {
        // 检查当前点是否在需要检查的三角形内
        // 首先排除当前需要检查三角形的三个顶点
        if (checkNode != curNode && checkNode->preNode != curNode && checkNode->nextNode != curNode)
        {
            // 存在点在当前三角形内，当前不是耳尖
            if (NavMath::IsPointInTriangle(
                 checkNode->point,
                 checkNode->nextNode->point,
                 checkNode->preNode->point,
                 curNode->point
                ))
                    return false;
        }
        
        curNode = curNode->nextNode;
    }
    while (curNode != firstNode);
    
    return true;
}

bool EarClipping()
{
    // 判断当前情况是否可以进行剪裁
    // 检查链表深度
    if (firstNode == nullptr)
        return false;

    // 收入最后一个三角形
    if (firstNode->nextNode->nextNode->nextNode == firstNode)
    {
        triangles.push_back(Triangle(firstNode->point, firstNode->preNode->point, firstNode->nextNode->point));
        firstNode = nullptr;
        return false;
    }
    
    // 取当前第一个耳尖，进行剪裁
    PointLinkNode* curNode = firstNode;
    do
    {
        // 针对当前行为进行剪裁
        if (IsPointEar(curNode))
        {
            // 在链表中去除当前点
            curNode->preNode->nextNode = curNode->nextNode;
            curNode->nextNode->preNode = curNode->preNode;
            // 追加分割好的三角形
            triangles.push_back(Triangle(curNode->point, curNode->preNode->point, curNode->nextNode->point));
            return true;
        }
        
        curNode = curNode->nextNode;
    } while(curNode != firstNode);
    
    return false;
}