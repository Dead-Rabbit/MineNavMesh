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

Vector3 startPoint  = Vector3(295,344, 0);
Vector3 endPoint    = Vector3(266,195, 0);
vector<Line> pathLines;
int drawFromToNum = -1;

int main(int argc, char* argv[])
{
    // 绘制裁剪前的图形
    initgraph(graphSize->x, graphSize->y);    // 创建绘图窗口，大小为 640x480 像素
    setbkmode(TRANSPARENT);     // 去掉文字背景颜色
    settextstyle(20, 0, L"微软雅黑");
    ClearBoard();
    ReDrawBoard();

    // 输入lines
    pathLines.push_back(Line(Vector3(583,395), Vector3(512,273)));
    pathLines.push_back(Line(Vector3(544,82), Vector3(512,273)));
    pathLines.push_back(Line(Vector3(544,82), Vector3(445,216)));
    pathLines.push_back(Line(Vector3(544,82), Vector3(409,259)));

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

Vector3 leftPoint = Vector3(0, 0);
Vector3 rightPoint = Vector3(0, 0);
Vector3 curPoint = Vector3(0, 0);
vector<Vector3> finalPath;
bool recordPoint = true;
void DoCheckOnStep()
{
    leftPoint = Vector3(0, 0);
    rightPoint = Vector3(0, 0);
    curPoint = startPoint;
    recordPoint = true;
    finalPath.clear();
    finalPath.push_back(startPoint);

    // 执行至第几步
    for(int i = 0; i < pathLines.size(); i++)
    {
        if (i > drawFromToNum)
            return;
        
        Line curLine = pathLines[i];
        Vector3 lineRight = curLine.from;
        Vector3 lineLeft = curLine.end;
        
        if (recordPoint)
        {
            leftPoint = lineLeft;
            rightPoint = lineRight;
            recordPoint = false;
            continue;
        }

        cout << "-----------------" << i << endl;

        // 检查left 或 right 是否越界, z 表达为，负数在Left，正数在Right
        Vector3 leftCrossEndRes = (lineLeft - curPoint).crossProduct(endPoint - curPoint);
        Vector3 rightCrossEndRes = (lineRight - curPoint).crossProduct(endPoint - curPoint);
        // 判断左点的情况
        Vector3 leftCrossLeftRes = (leftPoint - curPoint).crossProduct(lineLeft - curPoint);
        Vector3 rightCrossLeftRes = (rightPoint - curPoint).crossProduct(lineLeft - curPoint);
        if (leftCrossLeftRes.z > 0)
        {
            if (rightCrossLeftRes.z < 0)
            {
                leftPoint = lineLeft;
                cout << "Left inside" << endl;
            } else
            {
                // 当前情况为，线的left节点在目前right节点的right或直线上
                // 则记录当前right点为节点，并将left 和 right 节点置为当前线上的节点
                cout << "Left outside Of Right" << endl;
                finalPath.push_back(rightPoint);
                curPoint = rightPoint;
                // leftPoint = lineLeft;
                // rightPoint = lineRight;
                recordPoint = true;
            }
        }
        
        // 判断右点情况
        Vector3 leftCrossRightRes = (leftPoint - curPoint).crossProduct(lineRight - curPoint);
        Vector3 rightCrossRightRes = (rightPoint - curPoint).crossProduct(lineRight - curPoint);
        if (rightCrossRightRes.z < 0)
        {
            if (leftCrossRightRes.z > 0)
            {
                rightPoint = lineRight;
                cout << "Right inside" << endl;
            } else
            {
                // 当前情况为，线的Right节点在目前left节点的left或直线上
                // 则记录当前left点为节点，并将left 和 right 节点置为当前线上的节点
                cout << "Right outside" << endl;
                
                finalPath.push_back(leftPoint);
                curPoint = leftPoint;
                // leftPoint = lineLeft;
                // rightPoint = lineRight;
                recordPoint = true;
            }
        }
    }
    
    // 判断左点的情况
    Vector3 leftCrossEndRes = (leftPoint - curPoint).crossProduct(endPoint - curPoint);
    Vector3 rightCrossEndRes = (rightPoint - curPoint).crossProduct(endPoint - curPoint);
    cout << leftCrossEndRes.z << " - " << rightCrossEndRes.z << endl;
    if (rightCrossEndRes.z >= 0)
    {
        // 追加右侧侧的遗留内容
        finalPath.push_back(rightPoint);
    }
    
    if (leftCrossEndRes.z <= 0)
    {
        // 追加左侧的遗留内容
        finalPath.push_back(leftPoint);
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

    // 绘制起始点
    // setfillcolor(RED);
    // fillcircle(startPoint.x, startPoint.y, 2);
    // setfillcolor(GREEN);
    // fillcircle(endPoint.x, endPoint.y, 2);

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
    setlinecolor(BROWN);
    for (Vector3 pathPoint : finalPath)
    {
        line(prePoint.x, prePoint.y, pathPoint.x, pathPoint.y);
        prePoint = pathPoint;
    }
}