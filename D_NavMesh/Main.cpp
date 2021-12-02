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
float scale = 0.001;

// 绘制
void ClearBoard();
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
vector<Vector3> finalPathNodes;

Vector3 startPoint  = Vector3(264, 204, 10);
Vector3 endPoint    = Vector3(308, 291, 10);
vector<int> innerPathIndexs;

bool setStart = true;

bool dragSetLeft = false;
bool dragSetRight = false;

ClipTriangle* showTriangle = nullptr;
int step = 0;
bool CheckTriangleInfoMod = false;

int main(int argc, char* argv[])
{
    polygonNavMeshTool = PolygonNavMeshTool();
    
    vector<vector<Vector3>> outputSubjectPaths;    // 外边框路径
    vector<vector<Vector3>> outputClipPaths;       // 裁剪用路径
    
    // vector<Vector3> subjectPath = vector<Vector3>();
    // subjectPath.push_back(Vector3(190762, 13992.9, 205920));
    // subjectPath.push_back(Vector3(196140, 20616.9, 205920));
    // subjectPath.push_back(Vector3(190296, 14371.1, 205920));
    // subjectPath.push_back(Vector3(195674, 20995.1, 205920));
    // polygonNavMeshTool.AddPolygonOutsideContour(subjectPath);
    // outputSubjectPaths.push_back(subjectPath);
    
    vector<Vector3> subjectPath = vector<Vector3>();
    subjectPath.push_back(Vector3(100, 100, 10));
    subjectPath.push_back(Vector3(100, 300, 10));
    subjectPath.push_back(Vector3(300, 300, 10));
    subjectPath.push_back(Vector3(300, 100, 10));
    polygonNavMeshTool.AddPolygonOutsideContour(subjectPath);
    outputSubjectPaths.push_back(subjectPath);
    
    subjectPath = vector<Vector3>();
    subjectPath.push_back(Vector3(240, 100, 10));
    subjectPath.push_back(Vector3(240, 300, 10));
    subjectPath.push_back(Vector3(600, 200, 10));
    subjectPath.push_back(Vector3(600, 100, 10));
    polygonNavMeshTool.AddPolygonOutsideContour(subjectPath);
    outputSubjectPaths.push_back(subjectPath);
    
    vector<Vector3> clipPath = vector<Vector3>();
    int pathIndex = 0;
    
    clipPath.push_back(Vector3(258, 151, 10));
    clipPath.push_back(Vector3(281, 324, 10));
    clipPath.push_back(Vector3(324, 317, 10));
    clipPath.push_back(Vector3(297, 148, 10));
    pathIndex = polygonNavMeshTool.AddPolygonInsideContour(clipPath);
    innerPathIndexs.push_back(pathIndex);
    outputClipPaths.push_back(clipPath);
    
    clipPath = vector<Vector3>();
    clipPath.push_back(Vector3(387, 158, 10));
    clipPath.push_back(Vector3(368, 190, 10));
    clipPath.push_back(Vector3(421, 208, 10));
    clipPath.push_back(Vector3(458, 161, 10));
    clipPath.push_back(Vector3(436, 142, 10));
    pathIndex = polygonNavMeshTool.AddPolygonInsideContour(clipPath);
    innerPathIndexs.push_back(pathIndex);
    outputClipPaths.push_back(clipPath);
    
    clipPath = vector<Vector3>();
    clipPath.push_back(Vector3(489, 139, 10));
    clipPath.push_back(Vector3(478, 185, 10));
    clipPath.push_back(Vector3(508, 197, 10));
    clipPath.push_back(Vector3(528, 158, 10));
    clipPath.push_back(Vector3(516, 134, 10));
    pathIndex = polygonNavMeshTool.AddPolygonInsideContour(clipPath);
    innerPathIndexs.push_back(pathIndex);
    outputClipPaths.push_back(clipPath);
    //
    clipPath = vector<Vector3>();
    clipPath.push_back(Vector3(158, 174, 10));
    clipPath.push_back(Vector3(145, 230, 10));
    clipPath.push_back(Vector3(190, 241, 10));
    clipPath.push_back(Vector3(212, 200, 10));
    clipPath.push_back(Vector3(194, 173, 10));
    pathIndex = polygonNavMeshTool.AddPolygonInsideContour(clipPath);
    innerPathIndexs.push_back(pathIndex);
    outputClipPaths.push_back(clipPath);
    
    clipPath = vector<Vector3>();
    clipPath.push_back(Vector3(288, 131, 10));
    clipPath.push_back(Vector3(402, 167, 10));
    clipPath.push_back(Vector3(372, 75,  10));
    clipPath.push_back(Vector3(302, 62, 10));
    pathIndex = polygonNavMeshTool.AddPolygonInsideContour(clipPath);
    innerPathIndexs.push_back(pathIndex);
    outputClipPaths.push_back(clipPath);
    
    clipPath = vector<Vector3>();
    clipPath.push_back(Vector3(246, 65, 10));
    clipPath.push_back(Vector3(247, 185, 10));
    clipPath.push_back(Vector3(313, 188,  10));
    clipPath.push_back(Vector3(324, 73, 10));
    pathIndex = polygonNavMeshTool.AddPolygonInsideContour(clipPath);
    innerPathIndexs.push_back(pathIndex);
    outputClipPaths.push_back(clipPath);

    // vector<Vector3> subjectPath = vector<Vector3>();
    // subjectPath.push_back(Vector3(88, 87, 0));
    // subjectPath.push_back(Vector3(90, 410, 0));
    // subjectPath.push_back(Vector3(583, 395, 0));
    // subjectPath.push_back(Vector3(544, 82, 0));
    // polygonNavMeshTool.AddPolygonOutsideContour(subjectPath);
    // outputSubjectPaths.push_back(subjectPath);
    //
    // vector<Vector3> clipPath = vector<Vector3>();
    // clipPath.push_back(Vector3(445, 216, 0));
    // clipPath.push_back(Vector3(409, 259, 0));
    // clipPath.push_back(Vector3(432, 320, 0));
    // clipPath.push_back(Vector3(487, 331, 0));
    // clipPath.push_back(Vector3(512, 273, 0));
    // pathIndex = polygonNavMeshTool.AddPolygonInsideContour(clipPath);
    // innerPathIndexs.push_back(pathIndex);
    // outputClipPaths.push_back(clipPath);
    //
    // clipPath = vector<Vector3>();
    // clipPath.push_back(Vector3(194, 215, 0));
    // clipPath.push_back(Vector3(168, 274, 0));
    // clipPath.push_back(Vector3(241, 325, 0));
    // clipPath.push_back(Vector3(311, 272, 0));
    // clipPath.push_back(Vector3(293, 213, 0));
    // pathIndex = polygonNavMeshTool.AddPolygonInsideContour(clipPath);
    // innerPathIndexs.push_back(pathIndex);
    // outputClipPaths.push_back(clipPath);
    
#ifdef USE_EASYX_GRAPHICS
    
    // 绘制裁剪前的图形
    initgraph(graphSize->x, graphSize->y);    // 创建绘图窗口，大小为 640x480 像素
    setbkmode(TRANSPARENT);     // 去掉文字背景颜色
    settextstyle(20, 0, L"微软雅黑");
    
    BeginBatchDraw();
    // 绘制当前调试用
    ClearBoard();
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
                ClearBoard();
                if (step == 0)
                {
                    step++;
                    // 1. 进行三角化
                    vector<vector<ClipTriangle*>> triangleGroups = polygonNavMeshTool.GenerateFinalTriangles();
                    
                    // 绘制生成的三角形
                    for (vector<ClipTriangle*> triangles : triangleGroups)
                    {
                        setcolor(BLACK);
                        for (ClipTriangle* drawTriangle : triangles)
                        {
                            auto A = drawTriangle->A->point, B = drawTriangle->B->point, C = drawTriangle->C->point;
                            line(A.x, A.y, B.x, B.y);
                            line(B.x, B.y, C.x, C.y);
                            line(C.x, C.y, A.x, A.y);
                        }
                    }
                } else if (step == 1)
                {
                    step++;
                    // 2. 生成路径后，输入开始、结束点，生成路径
                    finalPathNodes = polygonNavMeshTool.FindPath(startPoint, endPoint);
                } else
                {
                    step++;
                    if (CheckTriangleInfoMod)
                    {
                        // 输出当前点击三角形的信息
                        showTriangle = nullptr;
                        vector<vector<ClipTriangle*>> triangleGroups = polygonNavMeshTool.GetGenTriangles();
                        for (auto triangle_group : triangleGroups)
                        {
                            for (auto triangle : triangle_group)
                            {
                                if (triangle->IsPointInTriangle(Vector3(m.x, m.y, 0)))
                                {
                                    showTriangle = triangle;
                                    break;
                                }
                            }
                        }
                    } else
                    {
                        const auto clickPos = Vector3(m.x, m.y, 0);
                        startPoint = clickPos;
                        finalPathNodes = polygonNavMeshTool.FindPath(startPoint, endPoint);
                        dragSetLeft = true;
                    }
                }
                ReDrawBoard();
            }break;
        case WM_LBUTTONUP:
            {
                dragSetLeft = false;
            }
            break;
        case WM_RBUTTONDOWN:
            {
                ClearBoard();
                std::cout << "(" << m.x << ", " << m.y << ")" << endl;
                const auto clickPos = Vector3(m.x, m.y, 0);
                endPoint = clickPos;
                finalPathNodes = polygonNavMeshTool.FindPath(startPoint, endPoint);
                ReDrawBoard();
                dragSetLeft = false;
                dragSetRight = true;
            }
            break;
        case WM_RBUTTONUP:
            {
                dragSetRight = false;
            }
            break;
        case WM_MBUTTONDOWN:
            {
                ClearBoard();
                if (polygonNavMeshTool.RemovePolygonInsideContour(innerPathIndexs[0]))
                {
                    finalPathNodes = polygonNavMeshTool.FindPath(startPoint, endPoint);
                }
                ReDrawBoard();
            }
            break;
        case WM_MOUSEMOVE:
            {
                if (step > 1)
                {
                    ClearBoard();
                    const auto pos = Vector3(m.x, m.y, 0);
                    if (dragSetLeft)
                    {
                        startPoint = pos;
                    } else if (dragSetRight)
                    {
                        endPoint = pos;
                    }
                    finalPathNodes = polygonNavMeshTool.FindPath(startPoint, endPoint);
                    ReDrawBoard();
                }
            }
            break;
        case WM_KEYDOWN:
            if (m.vkcode == VK_ESCAPE)
                return 0;	// Press ESC key to exit
        default: break;
        }
        FlushBatchDraw();
    }
    EndBatchDraw();
#else
    polygonNavMeshTool.GenerateFinalTriangles();
    int outTriangleNum = 1;
    for (OutsidePolygon* polygon : polygonNavMeshTool.GetOutsidePolygons())
    {
        const auto triangles = polygon->GetGenTriangles();
        if (triangles.size() != 0)
            for (const ClipTriangle* triangle : triangles)
            {
                std::cout << outTriangleNum++ << ": A(" << triangle->A->point.x << ", " << triangle->A->point.y
                    << ") - B(" << triangle->B->point.x << ", " << triangle->B->point.y
                    << ") - C(" << triangle->C->point.x << ", " << triangle->C->point.y << ")" << std::endl;
            }
    }
#endif
}

#ifdef USE_EASYX_GRAPHICS

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
    _stprintf_s(str, _T("点击左键继续"));
    outtextxy(0, 0, str);
    
    settextstyle(20, 0, L"微软雅黑");
    DrawTriangles();
    DrawPolygonsPoints();
    
    setfillcolor(BLUE);
    fillcircle(startPoint.x, startPoint.y, 3);
    setfillcolor(RED);
    fillcircle(endPoint.x, endPoint.y, 3);

    // 绘制最终生成的路线
    setcolor(RED);
    auto preLineStyle = new LINESTYLE();
    getlinestyle(preLineStyle);
    auto newLineStyle = new LINESTYLE();
    newLineStyle->style = preLineStyle->style;
    newLineStyle->thickness = 2;
    newLineStyle->puserstyle = preLineStyle->puserstyle;
    newLineStyle->userstylecount = preLineStyle->userstylecount;
    setlinestyle(newLineStyle);
    if (finalPathNodes.size() > 0)
    {
        for (int z = 0; z < finalPathNodes.size() - 1; z++)
        {
            const auto point = finalPathNodes[z];
            const auto nextPoint = finalPathNodes[z + 1];
            line(point.x, point.y, nextPoint.x, nextPoint.y);
        }
    }
    setlinestyle(preLineStyle);
    
    setcolor(BLACK);
    // 绘制所选三角形信息
    if (showTriangle != nullptr)
    {
        std::cout << "[" << showTriangle->num <<  "] 临近三角形: ";
        int showRadius = 2;
        for (const auto otherTriangle : showTriangle->GetLinkedClipTriangles())
        {
            std::cout << otherTriangle.first->num << " ";
            auto firstP = otherTriangle.second->A->point, secondP = otherTriangle.second->B->point;
            // 显示链接的起始点和结束点
            setlinecolor(GREEN);
            showRadius += 1.5;
            circle(firstP.x, firstP.y, showRadius);
            setlinecolor(RED);
            showRadius += 1.5;
            circle(secondP.x, secondP.y, showRadius);
        }
        std::cout << endl;
    }
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
            
            if (triangle != showTriangle)
                setfillcolor(0xF0FFF0); // 浅绿色
            else
                setfillcolor(0x00CED1); // 蓝色
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
        }
        
        for (ClipTriangle* triangle : triangles)
        {
            // 绘制内心
            setlinecolor(0xF0FFF0);
            setfillcolor(RED);
            const auto centerPos = triangle->centerPos;
            TCHAR str[25];
            _stprintf_s(str, _T("%d"), triangle->num);
            outtextxy(centerPos.x - 5, centerPos.y - 20, str);
        }
    }
}

#endif