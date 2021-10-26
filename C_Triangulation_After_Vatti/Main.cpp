#include "Sources/clipper.h"
#include "Sources/clipper_core.hpp"
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
void DrawPath(int pathNum, Path<double> path, COLORREF color);

#endif

std::vector<Path<double>> DoClipTest(ClipperD* clipperD);
std::vector<Path<double>> outputSubjectPaths;
std::vector<Path<double>> outputClipPaths;

int main(int argc, char* argv[])
{
    ClipperD* clipperD = new ClipperD();

    Path<double> subjectPath1 = Path<double>();
    subjectPath1.push_back(Point<double>(100, 100));
    subjectPath1.push_back(Point<double>(100, 300));
    subjectPath1.push_back(Point<double>(300, 300));
    subjectPath1.push_back(Point<double>(300, 100));
    clipperD->AddPath(subjectPath1, PathType::Subject, false);
    outputSubjectPaths.push_back(subjectPath1);
    
    Path<double> subjectPath2 = Path<double>();
    subjectPath2.push_back(Point<double>(310, 100));
    subjectPath2.push_back(Point<double>(310, 300));
    subjectPath2.push_back(Point<double>(600, 200));
    subjectPath2.push_back(Point<double>(600, 100));
    clipperD->AddPath(subjectPath2, PathType::Subject, false);
    outputSubjectPaths.push_back(subjectPath2);
    
    Path<double> clipPath1 = Path<double>();
    clipPath1.push_back(Point<double>(250, 90));
    clipPath1.push_back(Point<double>(281, 324));
    clipPath1.push_back(Point<double>(324, 317));
    clipPath1.push_back(Point<double>(292, 120));
    clipperD->AddPath(clipPath1, PathType::Clip, false);
    outputClipPaths.push_back(clipPath1);
    
    Path<double> clipPath2 = Path<double>();
    clipPath2.push_back(Point<double>(387, 158));
    clipPath2.push_back(Point<double>(368, 190));
    clipPath2.push_back(Point<double>(421, 208));
    clipPath2.push_back(Point<double>(458, 161));
    clipPath2.push_back(Point<double>(436, 142));
    clipperD->AddPath(clipPath2, PathType::Clip, false);
    outputClipPaths.push_back(clipPath2);
    
    Path<double> clipPath3 = Path<double>();
    clipPath3.push_back(Point<double>(489, 139));
    clipPath3.push_back(Point<double>(478, 185));
    clipPath3.push_back(Point<double>(508, 197));
    clipPath3.push_back(Point<double>(528, 158));
    clipPath3.push_back(Point<double>(516, 134));
    clipperD->AddPath(clipPath3, PathType::Clip, false);
    outputClipPaths.push_back(clipPath3);
    
    Path<double> clipPath4 = Path<double>();
    clipPath4.push_back(Point<double>(158, 174));
    clipPath4.push_back(Point<double>(145, 230));
    clipPath4.push_back(Point<double>(190, 241));
    clipPath4.push_back(Point<double>(212, 200));
    clipPath4.push_back(Point<double>(194, 173));
    clipperD->AddPath(clipPath4, PathType::Clip, false);
    outputClipPaths.push_back(clipPath4);
    
    Path<double> clipPath5 = Path<double>();
    clipPath5.push_back(Point<double>(348, 223));
    clipPath5.push_back(Point<double>(213, 267));
    clipPath5.push_back(Point<double>(223, 342));
    clipPath5.push_back(Point<double>(310, 348));
    clipPath5.push_back(Point<double>(365, 309));
    clipperD->AddPath(clipPath5, PathType::Clip, false);
    outputClipPaths.push_back(clipPath5);
    
    std::vector<Path<double>> resultPaths = DoClipTest(clipperD);
    
    return 0;
}

// 进行裁剪
std::vector<Path<double>> DoClipTest(ClipperD* clipperD)
{
    PathsD resultPaths = PathsD();
    // 获取最后输出path
    if (clipperD->Execute(ClipType::Difference, FillRule::Negative, resultPaths))
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

void DrawPath(int pathNum, Path<double> path, COLORREF color)
{
    auto points = path.data;
    for (int i = 0; i < points.size(); i++)
    {
        auto point = points[i];
        auto nextPoint = points[i == points.size() - 1 ? 0 : i + 1];
        
        setlinecolor(color);
        line(point.x, point.y, nextPoint.x, nextPoint.y);
        
        setcolor(BLACK);
        TCHAR str[25];
        if (i == 0)
        {
            _stprintf_s(str, _T("%d[%d]"), i + 1, pathNum);
            outtextxy(point.x - 20, point.y - 20, str);
        }
        else
        {
            _stprintf_s(str, _T("%d"), i + 1);
            outtextxy(point.x - 5, point.y - 20, str);
        }
        
    }
}

#endif