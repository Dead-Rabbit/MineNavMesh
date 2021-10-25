#define USE_EASYX_GTAPHICS      // 定义是否使用 EasyX 来进行输出

#include <vector>

#include "../Base/Vectors.h"
using namespace std;
using namespace ZXNavMesh;

#ifdef USE_EASYX_GTAPHICS
#include <graphics.h>
#include <conio.h>         //为了使用_getch()
#endif

int main()
{
    // 定义多边形的点，介于 EasyX 使用的是整型，此处尽量使用整型来测试
    std::vector<Vector2> edgePoints;
    edgePoints.push_back(Vector2(174, 316));
    edgePoints.push_back(Vector2(153,153));
    edgePoints.push_back(Vector2(262,144));
    edgePoints.push_back(Vector2(337,209));
    edgePoints.push_back(Vector2(379,132));
    edgePoints.push_back(Vector2(400,288));
    edgePoints.push_back(Vector2(293,325));
    
#ifdef USE_EASYX_GTAPHICS
    initgraph(640, 480);    // 创建绘图窗口，大小为 640x480 像素
    
    // 绘制或输出测试
    // setfillstyle(SOLID_FILL, RED);
    setcolor(RED);
    setfillcolor(RED);
    for (Vector2 point : edgePoints)
    {
        fillcircle(point.x, point.y, 3);
    }
#endif

    // 三角化
    
#ifdef USE_EASYX_GTAPHICS
    // 输出 图形
    _getch();              // 按任意键继续,回收任意一个字符
    closegraph();          // 关闭绘图窗口
#endif

    return 0;
}