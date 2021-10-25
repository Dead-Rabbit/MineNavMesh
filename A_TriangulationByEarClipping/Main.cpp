
#pragma comment(lib,"user32")
#pragma comment(lib,"gdi32")
#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
HWND WINAPI GetConsoleWindow();
int main(int argc,char *argv[])
{
    HWND hwnd;
    HDC hdc;
    HPEN hpen;
    hwnd = GetConsoleWindow();
    hdc = GetDC(hwnd);
    system("color F0");
    system("cls");
    TextOut(hdc,50,100,nullptr,12);
    hpen=CreatePen(PS_SOLID,1,RGB(255,0,0));
    SelectObject(hdc,hpen);
    MoveToEx(hdc,20,20,NULL);
    LineTo(hdc,200,300);
    DeleteObject(hpen);
    ReleaseDC(hwnd,hdc);
    getchar();
    return 0;
}