/**
 * 学习自： https://www.cnblogs.com/xignzou/p/3721494.html
*/

// 定义是否使用 EasyX 来进行输出
// 仅限Windows使用，mac或其他系统请注释掉当前定义
#define USE_EASYX_GTAPHICS

#include <vector>

#include "../Base/Vectors.h"
#include "Sources/PointLinkNode.h"

#ifdef USE_EASYX_GTAPHICS
#include <graphics.h>
#include <conio.h>      //为了使用_getch()
#endif

using namespace std;
using namespace ZXNavMesh;

// 记录原始点，逆时针顺序输入
std::vector<Vector2> edgePoints;
// 链表的起点
PointLinkNode* firstNode = nullptr;

int main()
{
    // 定义多边形的点，介于 EasyX 使用的是整型，此处尽量使用整型来测试
    edgePoints.push_back(Vector2(174, 316));
    edgePoints.push_back(Vector2(153,153));
    edgePoints.push_back(Vector2(262,144));
    edgePoints.push_back(Vector2(337,209));
    edgePoints.push_back(Vector2(379,132));
    edgePoints.push_back(Vector2(400,288));
    edgePoints.push_back(Vector2(293,325));
    const int pointSize = edgePoints.size();

    // 生成链表
    PointLinkNode* curNode = firstNode;
    PointLinkNode* preNode = firstNode;
    for (int i = 0; i < pointSize; i++)
    {
        if (firstNode == nullptr)
        {
            firstNode = new PointLinkNode(edgePoints[i]);
            curNode = firstNode;
            preNode = curNode;
            continue;
        }
        
        curNode->nextNode = new PointLinkNode(edgePoints[i]);
        preNode = curNode;
        curNode = curNode->nextNode;
        curNode->preNode = preNode;
    }
    curNode->nextNode = firstNode;
    firstNode->preNode = curNode;
    
#ifdef USE_EASYX_GTAPHICS
    initgraph(640, 480);    // 创建绘图窗口，大小为 640x480 像素
    setfillcolor(WHITE);
    solidrectangle(0, 0, 640, 480);
    
    // 测试生成的链表
    setlinecolor(BLACK);
    setfillcolor(RED);
    PointLinkNode* outCurNode = firstNode;
    if (outCurNode != nullptr)
        do
        {
            const auto point = outCurNode->point;
            fillcircle(point.x, point.y, 3);
            const auto nextPoint = outCurNode->nextNode->point;
            line(point.x, point.y, nextPoint.x, nextPoint.y);
            outCurNode = outCurNode->nextNode;
        } while(outCurNode != firstNode);
#endif

    // 三角化
    
#ifdef USE_EASYX_GTAPHICS
    // 输出 图形
    _getch();              // 按任意键继续,回收任意一个字符
    closegraph();          // 关闭绘图窗口
#endif

    return 0;
}