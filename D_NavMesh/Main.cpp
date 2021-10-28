#include "Sources/clipper.h"
#include "Sources/PolygonTriangulation.h"

#define USE_EASYX_GRAPHICS  // 是否使用 EasyX 进行输出，目前EasyX仅支持Windows平台

#ifdef USE_EASYX_GRAPHICS
#include <graphics.h>
#include <conio.h>      //为了使用_getch()
#endif

using namespace std;
using namespace clipperlib;
using namespace ZXNavMesh;

#ifdef USE_EASYX_GRAPHICS

// 屏幕大小
Vector2* graphSize = new Vector2(640, 480);

// 绘制
void ReDrawBoard();
void DrawTriangles();
void DrawPolygonsPoints();
void DrawPoints(PointLinkNode* firstNode);
void DrawPath(int pathNum, Path<double> path, COLORREF color);

#endif

// 裁剪行为
void DoPolygonVatti();

bool finishedFindPath = false;
std::vector<Path<double>> resultPaths;

// 裁剪用工具
ClipperD clipperD = ClipperD();
// 耳切法三角化工具
PolygonTriangulation triangulationTool;

int main(int argc, char* argv[])
{
    triangulationTool = PolygonTriangulation();
    
    vector<Path<double>> outputSubjectPaths;    // 外边框路径
    vector<Path<double>> outputClipPaths;       // 裁剪用路径
    Path<double> subjectPath1 = Path<double>();
    subjectPath1.push_back(Point<double>(100, 100));
    subjectPath1.push_back(Point<double>(100, 300));
    subjectPath1.push_back(Point<double>(300, 300));
    subjectPath1.push_back(Point<double>(300, 100));
    clipperD.AddPath(subjectPath1, PathType::Subject, false);
    outputSubjectPaths.push_back(subjectPath1);
    
    Path<double> subjectPath2 = Path<double>();
    subjectPath2.push_back(Point<double>(240, 100));
    subjectPath2.push_back(Point<double>(240, 300));
    subjectPath2.push_back(Point<double>(600, 200));
    subjectPath2.push_back(Point<double>(600, 100));
    clipperD.AddPath(subjectPath2, PathType::Subject, false);
    outputSubjectPaths.push_back(subjectPath2);
    
    Path<double> clipPath1 = Path<double>();
    clipPath1.push_back(Point<double>(258, 151));
    clipPath1.push_back(Point<double>(281, 324));
    clipPath1.push_back(Point<double>(324, 317));
    clipPath1.push_back(Point<double>(297, 148));
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
    
#ifdef USE_EASYX_GRAPHICS
    
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
                    DoPolygonVatti();
                    triangulationTool.EarClipping();
                } else
                {
                    
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
    DoPolygonVatti();
    triangulationTool.EarClipping();
    int outTriangleNum = 1;
    for (OutsidePolygon* polygon : triangulationTool.GetOutsidePolygons())
    {
        const auto triangles = polygon->GetGenTriangles();
        if (triangles.size() != 0)
            for (ClipTriangle* triangle : triangles)
            {
                std::cout << "三角形"<< ++outTriangleNum << ": A(" << triangle->A->point.x << ", " << triangle->A->point.y
                    << ") - B(" << triangle->B->point.x << ", " << triangle->B->point.y
                    << ") - C(" << triangle->C->point.x << ", " << triangle->C->point.y << ")" << endl;
            }
    }
#endif

    return 0;
}

void DoPolygonVatti()
{
    // 裁剪后输出的Path数据
    PathsD pathsD = PathsD();
    // 获取最后输出path
    if (clipperD.Execute(ClipType::Difference, FillRule::Negative, pathsD))
    {
        resultPaths = pathsD.data;
        // 将resultPaths 加入到三角化程序中
        // 获取所有路径点和对应的岛洞
        for(int i = 0; i < resultPaths.size(); i++)
        {
            auto path = resultPaths[i];
            path.Reverse();
            
            vector<Vector3> pathNodes;
            for (auto pathNode : path.data)
            {
                pathNodes.push_back(Vector3(pathNode.x, pathNode.y, 0));
            }
            // 判断当前线段是否为外边框和岛洞
            triangulationTool.AddPolygonPoints(pathNodes);
        }
    }
}

#ifdef USE_EASYX_GRAPHICS

void ReDrawBoard()
{
    cleardevice();
    setfillcolor(WHITE);
    solidrectangle(0, 0, graphSize->x, graphSize->y); // 填充背景色
    
    settextstyle(25, 0, L"微软雅黑");
    setcolor(BLACK);
    TCHAR str[25];
    _stprintf_s(str, _T("点击左键继续"));
    outtextxy(0, 0, str);
    
    settextstyle(20, 0, L"微软雅黑");
    DrawTriangles();
    DrawPolygonsPoints();
    
    for (int i = 0; i < resultPaths.size(); i++)
    {
        auto path = resultPaths[i];
        DrawPath(i+1, path, GREEN);
    }
}

void DrawPath(int pathNum, Path<double> path, COLORREF color)
{
    if (finishedFindPath)
        return;
    
    auto points = path.data;
    for (int i = 0; i < points.size(); i++)
    {
        const auto point = points[i];
        const auto nextPoint = points[i == points.size() - 1 ? 0 : i + 1];
        
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

// 使用EasyX 输出点和线
void DrawPolygonsPoints()
{
    for (const OutsidePolygon* polygon : triangulationTool.GetOutsidePolygons())
    {
        auto firstNode = polygon->GetFirstNode();
        DrawPoints(firstNode);
    }
}

void DrawPoints(PointLinkNode* firstNode)
{
    PointLinkNode* curNode = firstNode;
    if (curNode == nullptr)
        return;

    do
    {
        const Vector3 point = curNode->point;
        const Vector3 nextPoint = curNode->nextNode->point;
        
        // 绘制点
        setlinecolor(BLACK);
        setfillcolor(RED);
    
        fillcircle(point.x, point.y, 3);

        // 绘制点信息
        setcolor(BLACK);
        TCHAR str[25];
        if (curNode == firstNode)
            _stprintf_s(str, _T("*%d"), curNode->num, point.x, point.y);
        else
            _stprintf_s(str, _T("%d"), curNode->num, point.x, point.y);
        outtextxy(point.x - 5, point.y - 20, str);
    
        // 绘制线
        setlinecolor(BLACK);
        line(point.x, point.y, nextPoint.x, nextPoint.y);
        curNode = curNode->nextNode;
    } while(curNode != firstNode);
}

void DrawTriangles()
{
    for (const OutsidePolygon* polygon : triangulationTool.GetOutsidePolygons())
    {
        const auto triangles = polygon->GetGenTriangles();
        if (triangles.size() == 0)
            return;
        
        for (ClipTriangle* triangle : triangles)
        {
            setlinecolor(GREEN);
            setfillcolor(0xF0FFF0);
            const Vector3 A = triangle->A->point;
            const Vector3 B = triangle->B->point;
            const Vector3 C = triangle->C->point;
            
            line(A.x, A.y, B.x, B.y);
            line(B.x, B.y, C.x, C.y);
            line(C.x, C.y, A.x, A.y);
            const int points[] = {A.x, A.y, B.x, B.y, C.x, C.y};
            fillpoly(3, points);
            
            // 绘制内心
            setlinecolor(0xF0FFF0);
            setfillcolor(RED);
            const auto centerPos = triangle->centerPos;
            fillcircle(centerPos.x, centerPos.y, 2);
            
            setlinecolor(BLUE);
            const auto firstPos = triangle->centerPos;
            for (const ClipTriangle* otherTriangle : triangle->GetLinkedClipTriangles())
            {
                const auto secondPos = otherTriangle->centerPos;
                line(firstPos.x, firstPos.y, secondPos.x, secondPos.y);
            }
        }
    }
}

#endif