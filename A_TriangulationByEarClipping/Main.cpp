#include <graphics.h>
#include <iostream>

// int main(int argc, char* argv[])
// {
//     std::cout << "Hello Triangulation By Ear Clipping!" << std::endl;
//     return 0;
// }

#include <conio.h>         //为了使用_getch()
int main()
{
    initgraph(640, 480);   // 创建绘图窗口，大小为 640x480 像素
    circle(200, 200, 100); // 画圆，圆心(200, 200)，半径 100
    _getch();              // 按任意键继续,回收任意一个字符
    closegraph();          // 关闭绘图窗口
}