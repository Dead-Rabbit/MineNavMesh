#include "Sources/clipper.h"
#include "../Base/Graphes.h"

#define USE_EASYX_GTAPHICS  // 是否使用 EasyX 进行输出，目前EasyX仅支持Windows平台

#ifdef USE_EASYX_GTAPHICS
#include <graphics.h>
#include <conio.h>      //为了使用_getch()
#endif

using namespace std;
using namespace ZXNavMesh;
using namespace clipperlib;

#ifdef USE_EASYX_GTAPHICS

// 屏幕大小
Vector2* graphSize = new Vector2(640, 480);

// 绘制
void ClearDrawBoard();
void DrawPath(Path<double> path, COLORREF color);

#endif

std::vector<Path<double>> DoClipTest();
std::vector<Path<double>> outputSubjectPaths;
std::vector<Path<double>> outputClipPaths;

ClipperD clipperD = ClipperD();

int main(int argc, char* argv[])
{
    Path<double> subjectPath1 = Path<double>();
    subjectPath1.push_back(Point<double>(100, 100));
    subjectPath1.push_back(Point<double>(100, 300));
    subjectPath1.push_back(Point<double>(300, 300));
    subjectPath1.push_back(Point<double>(300, 100));
    clipperD.AddPath(subjectPath1, PathType::Subject, false);
    outputSubjectPaths.push_back(subjectPath1);
    
    Path<double> subjectPath2 = Path<double>();
    subjectPath2.push_back(Point<double>(310, 100));
    subjectPath2.push_back(Point<double>(310, 300));
    subjectPath2.push_back(Point<double>(600, 200));
    subjectPath2.push_back(Point<double>(600, 100));
    clipperD.AddPath(subjectPath2, PathType::Subject, false);
    outputSubjectPaths.push_back(subjectPath2);
    
    Path<double> clipPath1 = Path<double>();
    clipPath1.push_back(Point<double>(252, 190));
    clipPath1.push_back(Point<double>(163, 267));
    clipPath1.push_back(Point<double>(180, 327));
    clipPath1.push_back(Point<double>(297, 342));
    clipPath1.push_back(Point<double>(396, 293));
    clipPath1.push_back(Point<double>(387, 205));
    clipperD.AddPath(clipPath1, PathType::Clip, false);
    outputClipPaths.push_back(clipPath1);
    
#ifdef USE_EASYX_GTAPHICS
    // 绘制裁剪前的图形
    initgraph(graphSize->x, graphSize->y);    // 创建绘图窗口，大小为 640x480 像素
    setbkmode(TRANSPARENT);     // 去掉文字背景颜色

    // 绘制当前调试用
    ClearDrawBoard();
    for (auto path : outputSubjectPaths)
    {
        DrawPath(path, BLUE);
    }
    for (auto path : outputClipPaths)
    {
        DrawPath(path, RED);
    }
    
    ExMessage m;		// Define a message variable
    while(true)
    {
        // Get a mouse message or a key message
        m = getmessage(EM_MOUSE | EM_KEY);
        switch(m.message)
        {
        case WM_LBUTTONDOWN:
        {
            ClearDrawBoard();
            std::vector<Path<double>> resultPaths = DoClipTest();
            for (auto path : resultPaths)
            {
                DrawPath(path, GREEN);
            }
        }break;
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
    DoClipTest();
#endif

    return 0;
}

std::vector<Path<double>> DoClipTest()
{
    PathsD resultPaths = PathsD();
    // 获取最后输出path
    if (clipperD.Execute(ClipType::Difference, FillRule::EvenOdd, resultPaths))
    {
        return resultPaths.data;
    }
    
    return {};
}

#ifdef USE_EASYX_GTAPHICS

void ClearDrawBoard()
{
    cleardevice();
    setfillcolor(WHITE);
    solidrectangle(0, 0, graphSize->x, graphSize->y); // 填充背景色
}

void DrawPath(Path<double> path, COLORREF color)
{
    setlinecolor(color);
    auto points = path.data;
    for (int i = 0; i < points.size(); i++)
    {
        auto point = points[i];
        auto nextPoint = points[i == points.size() - 1 ? 0 : i + 1];
        line(point.x, point.y, nextPoint.x, nextPoint.y);
    }
}

#endif