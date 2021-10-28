#include "Sources/clipper.h"
#include "Sources/PolygonNavMeshTool.h"
#include "Sources/PolygonTriangulation.h"

#define USE_EASYX_GRAPHICS  // 是否使用 EasyX 进行输出，目前EasyX仅支持Windows平台

#ifdef USE_EASYX_GRAPHICS
#include <graphics.h>
#include <conio.h>      //为了使用_getch()
#endif

using namespace std;
using namespace clipperlib;
using namespace PolygonNavMesh;
using namespace NavMeshBase;

#ifdef USE_EASYX_GRAPHICS

// 屏幕大小
Vector2* graphSize = new Vector2(640, 480);

// 绘制
void ReDrawBoard();
void DrawTriangles();
void DrawPolygonsPoints();
void DrawPoints(PointLinkNode* firstNode);
void DrawPath(int pathNum, vector<Vector3> path, COLORREF color);

#endif

std::vector<Path<double>> resultPaths;

// 裁剪用工具
// ClipperD clipperD = ClipperD();
// 耳切法三角化工具

PolygonNavMeshTool polygonNavMeshTool;

int main(int argc, char* argv[])
{
    polygonNavMeshTool = PolygonNavMeshTool();
    
    vector<vector<Vector3>> outputSubjectPaths;    // 外边框路径
    vector<vector<Vector3>> outputClipPaths;       // 裁剪用路径
    
    vector<Vector3> subjectPath = vector<Vector3>();
    subjectPath.push_back(Vector3(100, 100, 0));
    subjectPath.push_back(Vector3(100, 300, 0));
    subjectPath.push_back(Vector3(300, 300, 0));
    subjectPath.push_back(Vector3(300, 100, 0));
    polygonNavMeshTool.AddPolygonOutsideContour(subjectPath);
    outputSubjectPaths.push_back(subjectPath);
    
    subjectPath = vector<Vector3>();
    subjectPath.push_back(Vector3(240, 100, 0));
    subjectPath.push_back(Vector3(240, 300, 0));
    subjectPath.push_back(Vector3(600, 200, 0));
    subjectPath.push_back(Vector3(600, 100, 0));
    polygonNavMeshTool.AddPolygonOutsideContour(subjectPath);
    outputSubjectPaths.push_back(subjectPath);
    
    vector<Vector3> clipPath = vector<Vector3>();
    clipPath.push_back(Vector3(258, 151, 0));
    clipPath.push_back(Vector3(281, 324, 0));
    clipPath.push_back(Vector3(324, 317, 0));
    clipPath.push_back(Vector3(297, 148, 0));
    polygonNavMeshTool.AddPolygonInsideContour(clipPath);
    outputClipPaths.push_back(clipPath);
    
    clipPath = vector<Vector3>();
    clipPath.push_back(Vector3(387, 158, 0));
    clipPath.push_back(Vector3(368, 190, 0));
    clipPath.push_back(Vector3(421, 208, 0));
    clipPath.push_back(Vector3(458, 161, 0));
    clipPath.push_back(Vector3(436, 142, 0));
    polygonNavMeshTool.AddPolygonInsideContour(clipPath);
    outputClipPaths.push_back(clipPath);

    clipPath = vector<Vector3>();
    clipPath.push_back(Vector3(489, 139, 0));
    clipPath.push_back(Vector3(478, 185, 0));
    clipPath.push_back(Vector3(508, 197, 0));
    clipPath.push_back(Vector3(528, 158, 0));
    clipPath.push_back(Vector3(516, 134, 0));
    polygonNavMeshTool.AddPolygonInsideContour(clipPath);
    outputClipPaths.push_back(clipPath);

    clipPath = vector<Vector3>();
    clipPath.push_back(Vector3(158, 174, 0));
    clipPath.push_back(Vector3(145, 230, 0));
    clipPath.push_back(Vector3(190, 241, 0));
    clipPath.push_back(Vector3(212, 200, 0));
    clipPath.push_back(Vector3(194, 173, 0));
    polygonNavMeshTool.AddPolygonInsideContour(clipPath);
    outputClipPaths.push_back(clipPath);
    
    clipPath = vector<Vector3>();
    clipPath.push_back(Vector3(288, 131, 0));
    clipPath.push_back(Vector3(402, 167, 0));
    clipPath.push_back(Vector3(372, 75,  0));
    clipPath.push_back(Vector3(302, 62, 0));
    polygonNavMeshTool.AddPolygonInsideContour(clipPath);
    outputClipPaths.push_back(clipPath);
    
    clipPath = vector<Vector3>();
    clipPath.push_back(Vector3(190, 83,0));
    clipPath.push_back(Vector3(174, 185, 0));
    clipPath.push_back(Vector3(240, 67, 0));
    polygonNavMeshTool.AddPolygonInsideContour(clipPath);
    outputClipPaths.push_back(clipPath);
    
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

    bool finishedFindPath = false;
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
                std::cout << "Left => (" << m.x << ", " << m.y << ")" << endl;
                if (!finishedFindPath)
                {
                    finishedFindPath = true;
                    std::vector<Path<double>> paths = polygonNavMeshTool.GenFinalTriangles();

                    for (Path<double> path : paths)
                    {
                        setcolor(BLACK);
                        Point<double> preP = path.data[0];
                        for (int j = 0; j < path.data.size(); j++)
                        {
                            auto point = path.data[j];
                            line(preP.x, preP.y, point.x, point.y);
                            preP = point;   
                        }
                        line(path.data[path.data.size() - 1].x, path.data[path.data.size() - 1].y,
                            path.data[0].x, path.data[0].y);
                    }
                } else
                {
                    // 生成路径后，输入开始、结束点，生成路径
                    Vector3 startPoint = Vector3(201, 286, 0);
                    Vector3 endPoint = Vector3(433, 219, 0);
                    vector<ClipTriangle*> pathTriangles = polygonNavMeshTool.FindPath(startPoint, endPoint);
                    if (pathTriangles.size() > 0)
                    {
                        setfillcolor(YELLOW);
                        for (int i = 0; i < pathTriangles.size(); i++)
                        {
                            auto triangle = pathTriangles[i];
                            Vector3 A = triangle->A->point;
                            Vector3 B = triangle->B->point;
                            Vector3 C = triangle->C->point;
                            const int points[] = {A.x, A.y, B.x, B.y, C.x, C.y};
                            fillpoly(3, points);
                            // line(pathNodes[i].x, pathNodes[i].y, pathNodes[i + 1].x, pathNodes[i + 1].y);
                        }
                    }
                    setfillcolor(BLUE);
                    fillcircle(startPoint.x, startPoint.y, 3);
                    setfillcolor(RED);
                    fillcircle(endPoint.x, endPoint.y, 3);
                }
            }break;
        case WM_RBUTTONDOWN:
            {
                std::cout << "Right => (" << m.x << ", " << m.y << ")" << endl;
            }
            break;
        case WM_KEYDOWN:
            if (m.vkcode == VK_ESCAPE)
                return 0;	// Press ESC key to exit
        default: break;
        }
    }
#else
    polygonNavMeshTool.GenFinalTriangles();
    int outTriangleNum = 1;
    for (OutsidePolygon* polygon : polygonNavMeshTool.GetOutsidePolygons())
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
}

void DrawPath(int pathNum, vector<Vector3> points, COLORREF color)
{
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
    for (const OutsidePolygon* polygon : polygonNavMeshTool.GetOutsidePolygons())
    {
        const auto firstNode = polygon->GetFirstNode();
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
    for (const OutsidePolygon* polygon : polygonNavMeshTool.GetOutsidePolygons())
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