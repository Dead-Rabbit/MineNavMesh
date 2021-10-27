#include "Sources/clipper.h"
#include "../Base/Graphes.h"
#include "../Base/NavMeshHelper.h"
#include "Sources/PolygonTriangulation.h"

#define USE_EASYX_GTAPHICS  // 是否使用 EasyX 进行输出，目前EasyX仅支持Windows平台

#ifdef USE_EASYX_GTAPHICS
#include <graphics.h>
#include <conio.h>      //为了使用_getch()
#endif

using namespace std;
using namespace clipperlib;
using namespace ZLNavMesh;

#ifdef USE_EASYX_GTAPHICS

// 屏幕大小
Vector2* graphSize = new Vector2(640, 480);

// 绘制
void ReDrawBoard();
void DrawTriangles();
void DrawPath(int pathNum, Path<double> path, COLORREF color);

#endif

std::vector<Path<double>> DoClipTest();
std::vector<Path<double>> outputSubjectPaths;
std::vector<Path<double>> outputClipPaths;

bool finishedFindPath = false;
std::vector<Path<double>> resultPaths;

ClipperD clipperD = ClipperD();
PolygonTriangulation triangulationTool;

int main(int argc, char* argv[])
{
    triangulationTool = PolygonTriangulation();
    
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
    clipPath1.push_back(Point<double>(250, 90));
    clipPath1.push_back(Point<double>(281, 324));
    clipPath1.push_back(Point<double>(324, 317));
    clipPath1.push_back(Point<double>(292, 120));
    clipperD.AddPath(clipPath1, PathType::Clip, false);
    outputClipPaths.push_back(clipPath1);
    
    Path<double> clipPath2 = Path<double>();
    clipPath2.push_back(Point<double>(387, 158));
    clipPath2.push_back(Point<double>(368, 190));
    clipPath2.push_back(Point<double>(421, 208));
    clipPath2.push_back(Point<double>(458, 161));
    clipPath2.push_back(Point<double>(436, 142));
    clipperD.AddPath(clipPath2, PathType::Clip, false);
    outputClipPaths.push_back(clipPath2);
    
    Path<double> clipPath3 = Path<double>();
    clipPath3.push_back(Point<double>(489, 139));
    clipPath3.push_back(Point<double>(478, 185));
    clipPath3.push_back(Point<double>(508, 197));
    clipPath3.push_back(Point<double>(528, 158));
    clipPath3.push_back(Point<double>(516, 134));
    clipperD.AddPath(clipPath3, PathType::Clip, false);
    outputClipPaths.push_back(clipPath3);
    
    Path<double> clipPath4 = Path<double>();
    clipPath4.push_back(Point<double>(158, 174));
    clipPath4.push_back(Point<double>(145, 230));
    clipPath4.push_back(Point<double>(190, 241));
    clipPath4.push_back(Point<double>(212, 200));
    clipPath4.push_back(Point<double>(194, 173));
    clipperD.AddPath(clipPath4, PathType::Clip, false);
    outputClipPaths.push_back(clipPath4);
    
    Path<double> clipPath5 = Path<double>();
    clipPath5.push_back(Point<double>(348, 223));
    clipPath5.push_back(Point<double>(213, 267));
    clipPath5.push_back(Point<double>(223, 342));
    clipPath5.push_back(Point<double>(310, 348));
    clipPath5.push_back(Point<double>(365, 309));
    clipperD.AddPath(clipPath5, PathType::Clip, false);
    outputClipPaths.push_back(clipPath5);
    
#ifdef USE_EASYX_GTAPHICS
    
    // 绘制裁剪前的图形
    initgraph(graphSize->x, graphSize->y);    // 创建绘图窗口，大小为 640x480 像素
    setbkmode(TRANSPARENT);     // 去掉文字背景颜色
    settextstyle(20, 0, L"微软雅黑");

    // 绘制当前调试用
    ReDrawBoard();
    int outputNum = 0;
    for (int i = 0; i < outputSubjectPaths.size(); i++)
    {
        auto path = outputSubjectPaths[i];
        outputNum++;
        DrawPath(outputNum, path, BLUE);
    }
    
    for (int i = 0; i < outputClipPaths.size(); i++)
    {
        auto path = outputClipPaths[i];
        outputNum++;
        DrawPath(outputNum, path, RED);
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
                ReDrawBoard();
                if (!finishedFindPath)
                {
                    finishedFindPath = true;
                    resultPaths = DoClipTest();
                    // 将resultPaths 加入到三角化程序中
                    // 获取所有路径点和对应的岛洞

                    setcolor(RED);
                    for(int i = 0; i < resultPaths.size(); i++)
                    {
                        auto path = resultPaths[i];
                        std::cout << "开始处理路线：" << i + 1 << endl;
                        path.Reverse();
                    
                        vector<Vector3> pathNodes;
                        for (auto pathNode : path.data)
                        {
                            pathNodes.push_back(Vector3(pathNode.x, pathNode.y, 0));
                        }

                        vector<vector<Vector3>> outsidePathNodes;
                        vector<vector<Vector3>> insidePathNodes;
                        
                        // 判断当前线段是否为外边框和岛洞
                        if (NavMeshHelper::IsTriangleOutside(pathNodes))
                        {
                            triangulationTool.AddPolygonOutPoints(pathNodes);
                        }
                        else
                        {
                            triangulationTool.AddPolygonInsidePoints(pathNodes);
                        }
                    }
                } else
                {
                    triangulationTool.OneStepEarClipping();
                }
                ReDrawBoard();
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
    if (clipperD.Execute(ClipType::Difference, FillRule::Negative, resultPaths))
    {
        return resultPaths.data;
    }
    
    return {};
}

#ifdef USE_EASYX_GTAPHICS

void ReDrawBoard()
{
    cleardevice();
    setfillcolor(WHITE);
    solidrectangle(0, 0, graphSize->x, graphSize->y); // 填充背景色
    
    for (int i = 0; i < resultPaths.size(); i++)
    {
        auto path = resultPaths[i];
        DrawPath(i+1, path, GREEN);
    }
    
    DrawTriangles();
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

void DrawTriangles()
{
    setfillcolor(0xF0FFF0);
    setlinecolor(GREEN);
    for (OutsidePolygon* polygon : triangulationTool.GetOutsidePolygons())
    {
        const auto triangles = polygon->GetGenTriangles();
        if (triangles.size() == 0)
            return;
        
        for (const Triangle triangle : triangles)
        {
            line(triangle.A.x, triangle.A.y, triangle.B.x, triangle.B.y);
            line(triangle.B.x, triangle.B.y, triangle.C.x, triangle.C.y);
            line(triangle.C.x, triangle.C.y, triangle.A.x, triangle.A.y);
            int points[] = {triangle.A.x, triangle.A.y, triangle.B.x, triangle.B.y, triangle.C.x, triangle.C.y};
            fillpoly(3, points);
        }
    }
}

#endif