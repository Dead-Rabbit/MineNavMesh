/**
 * 学习自： https://www.cnblogs.com/xignzou/p/3721494.html
*/

// 定义是否使用 EasyX 来进行输出
// 仅限Windows使用，mac或其他系统请注释掉当前定义
#define USE_EASYX_GTAPHICS

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

// 判断当前点是否为耳尖
bool IsPointEar(PointLinkNode* checkNode);

int main()
{
    // 定义多边形的点，介于 EasyX 使用的是整型，此处尽量使用整型来测试
    edgePoints.push_back(Vector3(55, 87, 0));
    edgePoints.push_back(Vector3(85, 355, 0));
    edgePoints.push_back(Vector3(305, 388, 0));
    edgePoints.push_back(Vector3(162, 319, 0));
    edgePoints.push_back(Vector3(414, 298, 0));
    edgePoints.push_back(Vector3(445, 381, 0));
    edgePoints.push_back(Vector3(454, 233, 0));
    edgePoints.push_back(Vector3(531, 154, 0));
    edgePoints.push_back(Vector3(440, 163, 0));
    edgePoints.push_back(Vector3(552, 69, 0));
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
    Vector2* graphSize = new Vector2(640, 480);
    initgraph(graphSize->x, graphSize->y);    // 创建绘图窗口，大小为 640x480 像素
    setfillcolor(WHITE);
    solidrectangle(0, 0, graphSize->x, graphSize->y); // 填充背景色
    setbkmode(TRANSPARENT);     // 去掉文字背景颜色
    
    // 测试生成的链表
    PointLinkNode* outCurNode = firstNode;
    if (outCurNode != nullptr)
        do
        {
            // std::cout<< "点" << outCurNode->num << "->" << outCurNode->nextNode->num << endl;
            // 绘制点
            const auto point = outCurNode->point;
            setlinecolor(BLACK);
            
            // 红色代表为凹角
            // 黄色代表为凸角，但不是耳尖
            // 绿色代表耳尖
            if (IsPointEar(outCurNode))
                setfillcolor(YELLOW);
            else
                if (outCurNode->IsPointConvex())
                    setfillcolor(GREEN);
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
#endif

    // 三角化
    
#ifdef USE_EASYX_GTAPHICS
    
    ExMessage m;		// Define a message variable
    while(true)
    {
        // Get a mouse message or a key message
        m = getmessage(EM_MOUSE | EM_KEY);
        switch(m.message)
        {
            case WM_LBUTTONDOWN:
                std::cout << "(" << m.x << ", " << m.y << ")" << endl;
                break;
            case WM_KEYDOWN:
                if (m.vkcode == VK_ESCAPE)
                    return 0;	// Press ESC key to exit
            default: break;
        }
    }
    
    closegraph();          // 关闭绘图窗口
#endif
    return 0;
}

// 判断这个点是否为耳尖
bool IsPointEar(PointLinkNode* checkNode)
{
    // 首先排除当前不是一个凸边的情况
    if (!checkNode->IsPointConvex())
        return false;
    
    // 检查当前三角形内是否包含其他点
    PointLinkNode* curNode = firstNode;
    do
    {
        // 检查当前点是否在需要检查的三角形内
        // 首先排除当前需要检查三角形的三个顶点
        if (checkNode != curNode && checkNode->preNode != curNode && checkNode->preNode != curNode)
        {
            // 存在点在当前三角形内，当前不是耳尖
            if (NavMath::IsInTrigon(
                curNode->point,
                checkNode->point,
                checkNode->preNode->point,
                checkNode->nextNode->point
                ))
                    return false;
        }
        
        curNode = curNode->nextNode;
    }
    while (curNode != firstNode);
    
    return true;
}