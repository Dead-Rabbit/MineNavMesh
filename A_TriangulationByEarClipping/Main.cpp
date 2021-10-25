#define USE_EASYX_GTAPHICS      // 定义是否使用 EasyX 来进行输出

#include <vector>
using namespace std;

#ifdef USE_EASYX_GTAPHICS
#include <graphics.h>
#include <conio.h>         //为了使用_getch()
#endif

int main()
{
    // 定义多边形的点
    
#ifdef USE_EASYX_GTAPHICS
    // 绘制或输出测试
    initgraph(640, 480);    // 创建绘图窗口，大小为 640x480 像素
    // circle(200, 200, 100);   // 画圆，圆心(200, 200)，半径 100
#endif

    // 三角化
    
#ifdef USE_EASYX_GTAPHICS
    // 输出 图形
    _getch();              // 按任意键继续,回收任意一个字符
    closegraph();          // 关闭绘图窗口
#endif

    return 0;
}