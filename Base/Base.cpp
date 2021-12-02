#include <cstdio>

template<typename T>
T min(const T& left, const T& right) 
{
	if (left < right) {
		return left;
	}
	if (right < left) {
		return right;
	}
	if (right ==left)
	{
		return (T)0;
	}
}
template<typename T>
T max(const T& left, const T& right)
{
	if (left < right) {
		return right;
	}
	if (right < left) {
		return left;
	}
	if (right ==left)
	{
		return (T)0;
	}
}
struct Point
{
	double x;
	double y;
 
	Point(double paramx,double paramy)
	{
		this->x = paramx;
		this->y = paramy;
	}
 
	Point()
	{
		this->x = 0;
		this->y = 0;
	}
};
 
bool PtInPolygon(Point p, Point* ptPolygon, int nCount)
{
	// 交点个数  
 
	int nCross = 0;
 
	for (int i = 0; i < nCount; i++)
	{
 
		Point p1 = ptPolygon[i];
 
		Point p2 = ptPolygon[(i + 1) % nCount];// 最后一个点与第一个点连线  
 
		if (p1.y == p2.y)
			continue;
		if (p.y < min<double>(p1.y, p2.y))
			continue;
		if (p.y >= max<double>(p1.y, p2.y))
			continue;
		// 求交点的x坐标  
 
		double x = (double)(p.y - p1.y) * (double)(p2.x - p1.x) / (double)(p2.y - p1.y) + p1.x;
 
		// 只统计p1p2与p向右射线的交点  
		if (x > p.x)
		{
			nCross++;
		}
 
	}
 
	// 交点为偶数，点在多边形之外  
 
	return (nCross % 2 == 1);
 
}
 
 
int main()
{
	Point* arr = new Point[4];
 
	arr[0] = Point(-400, 400);
	arr[1] = Point(-400, -400);
	arr[2] = Point(0, 0);
	arr[3] = Point(400, 0);
 
	Point test(0,40);
	bool isinregion = PtInPolygon(test, arr,4);
	printf("is in regon %d", isinregion);
	return 0;
}