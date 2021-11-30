#include "../Base/Vectors.h"
#include <graphics.h>
#include <conio.h>      //为了使用_getch()
#include <vector>

NavMeshBase::Vector2* graphSize = new NavMeshBase::Vector2(640, 480);

std::vector<NavMeshBase::Vector2> points;
NavMeshBase::Vector2 mainPoint;

int GEN_MIN_X = 20, GEN_MAX_X = 600;
int GEN_MIN_Y = 20, GEN_MAX_Y = 440;

void ClearBoard();
void ReDrawBoard();

/**
 *  测试比较 KDTree 和 普通方式 寻找Neighborhood 的效率
 */
int main(int argc, char* argv[])
{
    // Use the time function to get a "seed” value for srand
    const unsigned seed = time(0);
    srand(seed);
    const int genPointN = 200;
    points.clear();
    for (int i = 0; i < genPointN; i++)
    {
        const auto genX = rand() % (GEN_MAX_X - GEN_MIN_X + 1) + GEN_MIN_X;
        const auto genY = rand() % (GEN_MAX_Y - GEN_MIN_Y + 1) + GEN_MIN_Y;
        points.push_back(NavMeshBase::Vector2(genX, genY));
    }
    
    mainPoint = NavMeshBase::Vector2(rand() % (GEN_MAX_X - GEN_MIN_X + 1) + GEN_MIN_X,
        rand() % (GEN_MAX_Y - GEN_MIN_Y + 1) + GEN_MIN_Y);

    // 绘制裁剪前的图形
    initgraph(graphSize->x, graphSize->y);    // 创建绘图窗口，大小为 640x480 像素
    setbkmode(TRANSPARENT);     // 去掉文字背景颜色
    settextstyle(20, 0, L"微软雅黑");
    
    // 绘制当前调试用
    ClearBoard();
    ReDrawBoard();
    
    ExMessage m;		// Define a message variable
    while(true)
    {
        // Get a mouse message or a key message
        m = getmessage(EM_MOUSE | EM_KEY);
        switch(m.message)
        {
        case WM_LBUTTONDOWN:
            {
                // Test Length Compare To Find Neighborhood
    
                // Test KDTrees Find Neighborhood
                
                ReDrawBoard();
            }
            break;
        default:
            break;
        }
    }
}

void ClearBoard()
{
    cleardevice();
    setfillcolor(WHITE);
    solidrectangle(0, 0, graphSize->x, graphSize->y); // 填充背景色
}

void ReDrawBoard()
{
    setfillcolor(BLACK);
    for (auto point : points)
    {
        fillcircle(point.x, point.y, 2);
    }
    
    setfillcolor(RED);
    fillcircle(mainPoint.x, mainPoint.y, 2);
}