#include <chrono>

#include "../Base/Vectors.h"
#include <graphics.h>
#include <conio.h>      //为了使用_getch()
#include <vector>

#include <ctime>
#include <time.h>

#include "KDTree.h"

NavMeshBase::Vector2* graphSize = new NavMeshBase::Vector2(640, 480);

std::vector<NavMeshBase::Vector2> points;
NavMeshBase::Vector2 mainPoint;

int findNeighborNum = 1;

float minDis = 100;

int GEN_MIN_X = 20, GEN_MAX_X = 600;
int GEN_MIN_Y = 20, GEN_MAX_Y = 440;

int methodType = 0; // 0 为普通，1为KDTree
std::chrono::microseconds duration;

KDTree tree;

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
    const int genPointN = 2000;
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
    
    BeginBatchDraw();
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
                ClearBoard();
                ReDrawBoard();

                if (methodType == 0) methodType = 1; else methodType = 0;

                if (methodType == 1)
                {
                    // auto start = std::chrono::steady_clock::now();

                    // 构建树
                    pointVec point_vec = pointVec();
                    for (auto point : points)
                    {
                        point_t point_ts = point_t();
                        point_ts.push_back(point.x);
                        point_ts.push_back(point.y);
                        point_vec.push_back(point_ts);
                    }
                    tree = KDTree(point_vec);
                    // point_t main_point = point_t();
                    // main_point.push_back(mainPoint.x);
                    // main_point.push_back(mainPoint.y);
                    // auto findNeighborPoints = tree.neighborhood_points(main_point, minDis);
                    
                    // auto finish = std::chrono::steady_clock::now();
                    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(finish - start);
                    // std::cout << "Find By KDTree " << duration.count() << " ms" << std::endl;
                    //
                }
                
                // 测试遍历法
                // {
                //     auto start = std::chrono::steady_clock::now();
                //
                //     std::vector<NavMeshBase::Vector2> neighbors;
                //     float squ_dis = minDis * minDis;
                //     // 使用距离简单对比选择多个最近的
                //     for (auto point : points)
                //     {
                //         auto dis = (point - mainPoint).squaredLength();
                //         if (dis < squ_dis)
                //         {
                //             neighbors.push_back(point);
                //         }
                //     }
                //     
                //     auto finish = std::chrono::steady_clock::now();
                //     auto duration = std::chrono::duration_cast<std::chrono::microseconds>(finish - start);
                //     std::cout << "Find By Distance " << duration.count() << " ms" << std::endl;
                //     
                //     setfillcolor(GREEN);
                //     for (const auto point : neighbors)
                //     {
                //         fillcircle(point.x, point.y, 3);
                //     }
                // }

            }
            break;
        case WM_RBUTTONDOWN:
            {
                // // 测试 KDTree
                // {
                //     auto start = std::chrono::steady_clock::now();
                //
                //     // 构建树
                //     pointVec point_vec = pointVec();
                //     for (auto point : points)
                //     {
                //         point_t point_ts = point_t();
                //         point_ts.push_back(point.x);
                //         point_ts.push_back(point.y);
                //         point_vec.push_back(point_ts);
                //     }
                //     tree = KDTree(point_vec);
                //     point_t main_point = point_t();
                //     main_point.push_back(mainPoint.x);
                //     main_point.push_back(mainPoint.y);
                //     auto findNeighborPoints = tree.neighborhood_points(main_point, minDis);
                //     
                //     auto finish = std::chrono::steady_clock::now();
                //     auto duration = std::chrono::duration_cast<std::chrono::microseconds>(finish - start);
                //     std::cout << "Find By KDTree " << duration.count() << " ms" << std::endl;
                //     
                //     setfillcolor(YELLOW);
                //     for (auto point : findNeighborPoints)
                //     {
                //         fillcircle(point[0], point[1], 3);
                //     }
                // }
            }
            break;
        case WM_MOUSEMOVE:
            {
                ClearBoard();
                ReDrawBoard();
                mainPoint = NavMeshBase::Vector2(m.x, m.y);
                
                auto start = std::chrono::steady_clock::now();

                if (methodType == 0)
                {
                    std::vector<NavMeshBase::Vector2> neighbors;
                    float squ_dis = minDis * minDis;
                    // 使用距离简单对比选择多个最近的
                    for (auto point : points)
                    {
                        auto dis = (point - mainPoint).squaredLength();
                        if (dis < squ_dis)
                        {
                            neighbors.push_back(point);
                        }
                    }
                    
                    setfillcolor(GREEN);
                    for (const auto point : neighbors)
                    {
                        fillcircle(point.x, point.y, 3);
                    }
                } else
                {
                    point_t main_point = point_t();
                    main_point.push_back(mainPoint.x);
                    main_point.push_back(mainPoint.y);
                    auto findNeighborPoints = tree.neighborhood_points(main_point, minDis);
                    
                    setfillcolor(YELLOW);
                    for (auto point : findNeighborPoints)
                    {
                        fillcircle(point[0], point[1], 3);
                    }
                }
                
                auto finish = std::chrono::steady_clock::now();
                duration = std::chrono::duration_cast<std::chrono::microseconds>(finish - start);
                std::cout << duration.count() << " ms" << std::endl;
            }
            break;
        default:
            break;
        }
        FlushBatchDraw();
    }
    EndBatchDraw();
}

void ClearBoard()
{
    cleardevice();
    setfillcolor(WHITE);
    solidrectangle(0, 0, graphSize->x, graphSize->y); // 填充背景色
}

void ReDrawBoard()
{
    settextstyle(25, 0, L"微软雅黑");
    setcolor(BLACK);
    TCHAR str[25];
    _stprintf_s(str, _T("Current Method: %d", methodType));
    outtextxy(0, 0, str);
    
    setfillcolor(BLACK);
    for (const auto point : points)
    {
        fillcircle(point.x, point.y, 2);
    }
    
    setfillcolor(BLUE);
    fillcircle(mainPoint.x, mainPoint.y, 4);
}