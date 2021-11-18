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
    Vector3(double x, double y, double z = 0)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }
    
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    
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
Vector3 startPoint  = Vector3(321, 183, 0);
Vector3 endPoint    = Vector3(208, 353, 0);

int main(int argc, char* argv[])
{
    // 输入lines
    pathLines.push_back(Line(Vector3(293,213),Vector3(544,82)));
    pathLines.push_back(Line(Vector3(311,272),Vector3(544,82)));
    pathLines.push_back(Line(Vector3(311,272),Vector3(409,259)));
    pathLines.push_back(Line(Vector3(311,272),Vector3(432,320)));
    pathLines.push_back(Line(Vector3(241,325),Vector3(432,320)));
    pathLines.push_back(Line(endPoint, endPoint));

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

Vector3 curPoint = Vector3(0, 0);
Vector3 rightLeg;
Vector3 leftLeg;
auto curPointIndex = 0, leftLegIndex = 0, rightLegIndex = 0;
vector<Vector3> finalPath;
// funnel-algorithm
// https://github.com/TheGoozah/AIProgramming
void DoCheckOnStep()
{
    // 准备
    finalPath.clear();
    finalPath.push_back(startPoint);
    rightLegIndex = 0;

    curPoint = startPoint;
    rightLeg = pathLines[0].from - curPoint;
    leftLeg = pathLines[0].end - curPoint;
    curPointIndex = 0; leftLegIndex = 0; rightLegIndex = 0;

    // 执行至第几步
    for(int i = 0; i < pathLines.size(); i++)
    {
        if (i > drawFromToNum)
            return;

        auto curLine = pathLines[i];

        auto newRightLeg = curLine.from - curPoint;
        auto cpTightenFunnel = newRightLeg.crossProduct(rightLeg);
        if (cpTightenFunnel.z >= 0.0f)
        {
            auto cpDenegrateFunnel = newRightLeg.crossProduct(leftLeg);
            if (cpDenegrateFunnel.z < 0.0f) //No overlap, tighten!
            {
                rightLeg = newRightLeg;
                rightLegIndex = i;
            }
            else
            {
                curPoint = curPoint + leftLeg;
                curPointIndex = leftLegIndex;
                unsigned int newIt = curPointIndex + 1;
                leftLegIndex = newIt;
                rightLegIndex = newIt;
                i = newIt;

                //Store point
                finalPath.push_back(curPoint);

                //Calculate new legs (if not the end)
                if (newIt < pathLines.size())
                {
                    rightLeg = pathLines[rightLegIndex].from - curPoint;
                    leftLeg = pathLines[leftLegIndex].end - curPoint;
                    continue; //Restart
                }
            }
        }

        auto newLeftLeg = curLine.end - curPoint;
        cpTightenFunnel = newLeftLeg.crossProduct(leftLeg);
        if (cpTightenFunnel.z <= 0.0f) //Move inwards
        {
            auto cpDenegrateFunnel = newLeftLeg.crossProduct(rightLeg);
            if (cpDenegrateFunnel.z > 0.0f) //No overlap, tighten!
            {
                leftLeg = newLeftLeg;
                leftLegIndex = i;
            }
            else
            {
                //Rightleg becomes new curPoint point
                curPoint = curPoint + rightLeg;
                curPointIndex = rightLegIndex;
                unsigned int newIt = curPointIndex + 1;
                leftLegIndex = newIt;
                rightLegIndex = newIt;
                i = newIt;
                //Store point
                finalPath.push_back(curPoint);
                //Calculate new legs (if not the end)
                if (newIt < pathLines.size())
                {
                    //Calculate new legs (if not the end)
                    rightLeg = pathLines[rightLegIndex].from - curPoint;
                    leftLeg = pathLines[leftLegIndex].end - curPoint;
                }
            }
        }
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

    // 绘制起止点
    setfillcolor(RED);
    fillcircle(startPoint.x, startPoint.y, 5);
    setfillcolor(GREEN);
    fillcircle(endPoint.x, endPoint.y, 6);

    // 绘制拐角法
    setfillcolor(BLACK);
    setlinecolor(BLACK);
    circle(curPoint.x, curPoint.y, 6);
    
    setfillcolor(GREEN);
    setlinecolor(GREEN);
    fillcircle(curPoint.x + leftLeg.x, curPoint.y + leftLeg.y, 4);
    line(curPoint.x, curPoint.y, curPoint.x + leftLeg.x, curPoint.y + leftLeg.y);
    
    setfillcolor(RED);
    setlinecolor(RED);
    fillcircle(curPoint.x + rightLeg.x, curPoint.y + rightLeg.y, 4);
    line(curPoint.x, curPoint.y, curPoint.x + rightLeg.x, curPoint.y + rightLeg.y);

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