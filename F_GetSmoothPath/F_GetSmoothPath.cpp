#include <iostream>

#include <graphics.h>
#include <conio.h>      //为了使用_getch()
#include <vector>

namespace PolygonNavMesh
{
    class ClipLine;
}

using namespace std;

class Vector3
{
public:
    Vector3(){};
    Vector3(float x, float y, float z = 0)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }
    
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
    
    inline Vector3 crossProduct(const Vector3& rkVector) const { return Vector3(y * rkVector.z - z * rkVector.y, z * rkVector.x - x * rkVector.z, x * rkVector.y - y * rkVector.x); }
    inline bool operator == ( const Vector3& rkVector ) const { return ( x == rkVector.x && y == rkVector.y && z == rkVector.z ); }
    inline bool operator != ( const Vector3& rkVector ) const { return ( x != rkVector.x || y != rkVector.y || z != rkVector.z ); }
    inline Vector3 operator + ( const Vector3& rkVector ) const { return Vector3(x + rkVector.x, y + rkVector.y, z + rkVector.z); }
    inline Vector3 operator - ( const Vector3& rkVector ) const { return Vector3(x - rkVector.x, y - rkVector.y, z - rkVector.z); }
    inline Vector3 operator * ( const double fScalar ) const { return Vector3(x * fScalar, y * fScalar, z * fScalar); }
    inline Vector3 operator / ( const double fScalar ) const { return Vector3(x / fScalar, y / fScalar, z / fScalar);  }
    inline const Vector3& operator + () const { return *this; }
    inline Vector3 operator - () const { return Vector3(-x, -y, -z); }
    inline friend Vector3 operator * ( const double fScalar, const Vector3& rkVector ) { return Vector3(fScalar * rkVector.x, fScalar * rkVector.y, fScalar * rkVector.z); }
};

class Line
{
public:
    Line(Vector3 from, Vector3 end)
    {
        this->from = from;
        this->end = end;
    }
    
    Vector3 from, end;
};

Vector3* graphSize = new Vector3(640, 480);

// 屏幕大小
void ClearBoard()
{
    cleardevice();
    setfillcolor(WHITE);
    solidrectangle(0, 0, graphSize->x, graphSize->y); // 填充背景色
}
void DoCheckOnStep();
void ReDrawBoard();

vector<Line> pathLines;
int drawFromToNum = -1;
Vector3 startPoint  = Vector3(204, 200, 0);
Vector3 endPoint    = Vector3(179, 305, 0);

int main(int argc, char* argv[])
{
    // 输入lines
    pathLines.push_back(Line(Vector3(88,87),Vector3(194,215)));
    pathLines.push_back(Line(Vector3(88,87),Vector3(168,274)));
    pathLines.push_back(Line(Vector3(90,410),Vector3(168,274)));

    // 绘制裁剪前的图形
    initgraph(graphSize->x, graphSize->y);    // 创建绘图窗口，大小为 640x480 像素
    setbkmode(TRANSPARENT);     // 去掉文字背景颜色
    settextstyle(20, 0, L"微软雅黑");
    ClearBoard();
    ReDrawBoard();

    // 拿到起始点和结束点
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

                    drawFromToNum++;
                    if (drawFromToNum >= pathLines.size())
                        drawFromToNum = 0;

                    DoCheckOnStep();
                    ReDrawBoard();
                }
                break;
        }
    }
    
    return 0;
}

Vector3 rightLeg;
Vector3 leftLeg;
Vector3 curPoint = Vector3(0, 0);
int recordRightIndex = 0;
int recordLeftIndex = 0;
vector<Vector3> finalPath;
bool recordPoint = true;
// funnel-algorithm
// https://github.com/TheGoozah/AIProgramming
void DoCheckOnStep()
{
    // 准备
    finalPath.clear();
    finalPath.push_back(startPoint);

    if (pathLines.size() == 0)
        return;
    
    curPoint = startPoint;
    rightLeg = pathLines[0].from - curPoint;
    leftLeg = pathLines[0].end - curPoint;
    
    // 执行至第几步
    for(int i = 0; i < pathLines.size(); i++)
    {
        if (i > drawFromToNum)
            return;
        
        auto curLine = pathLines[i];
        
        auto newRightLeg = curLine.from - curPoint;
        // auto cpTightenFunnel = 
        
    }
    
    finalPath.push_back(endPoint);
}

void ReDrawBoard()
{
    settextstyle(25, 0, L"微软雅黑");
    setcolor(BLACK);
    TCHAR str[25];
    _stprintf_s(str, _T("点击左键继续"));
    outtextxy(0, 0, str);

    // 绘制默认线
    setlinecolor(BLACK);
    for (int i = 0; i < pathLines.size(); i++)
    {
        Line origLine = pathLines[i];
        
        Vector3 from = origLine.from;
        Vector3 end = origLine.end;
        line(from.x, from.y, end.x, end.y);

        if (drawFromToNum == i)
        {
            setcolor(RED);
            circle(from.x, from.y, 5);

            setcolor(GREEN);
            circle(end.x, end.y, 3);
        }
    }

    // 绘制拐角法
    setfillcolor(BLACK);
    setlinecolor(BLACK);
    circle(curPoint.x, curPoint.y, 6);
    
    setfillcolor(GREEN);
    setlinecolor(GREEN);
    fillcircle(leftPoint.x, leftPoint.y, 4);
    line(curPoint.x, curPoint.y, leftPoint.x, leftPoint.y);
    
    setfillcolor(RED);
    setlinecolor(RED);
    fillcircle(rightPoint.x, rightPoint.y, 4);
    line(curPoint.x, curPoint.y, rightPoint.x, rightPoint.y);

    // 绘制平滑后的内容
    Vector3 prePoint = startPoint;
    setlinecolor(RED);
    auto preLineStyle = new LINESTYLE();
    getlinestyle(preLineStyle);
    auto newLineStyle = new LINESTYLE();
    newLineStyle->style = preLineStyle->style;
    newLineStyle->thickness = 2;
    newLineStyle->puserstyle = preLineStyle->puserstyle;
    newLineStyle->userstylecount = preLineStyle->userstylecount;
    setlinestyle(newLineStyle);
    for (Vector3 pathPoint : finalPath)
    {
        line(prePoint.x, prePoint.y, pathPoint.x, pathPoint.y);
        prePoint = pathPoint;
    }
    setlinestyle(preLineStyle);
}