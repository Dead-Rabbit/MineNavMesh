#pragma once

#include <vector>

#include "Vectors.h"

using namespace NavMeshBase;

namespace ZXNavMesh
{
	class NavMath
	{
	public:
		NavMath(){};

		// 绝对值
		static double Abs(double value)
		{
			if (value < 0)
				return -value;
			
			return value;
		}

		static double Acos(double num)
		{
			return std::acos(num);
		}

		static double Sin(double num)
		{
			return std::sin(num);
		}

		static double Cos(double num)
		{
			return std::cos(num);
		}
		
		// 点乘
		static double Dot(Vector3 line1, Vector3 line2)
		{
			return line1.x * line2.x + line1.y * line2.y;
		}

		// 叉乘
		static Vector3 Cross(Vector3 line1, Vector3 line2)
		{
			return Vector3(
				line1.y * line2.z - line1.z * line2.y,
				line1.z * line2.x - line1.x * line2.z,
				line1.x * line2.y - line1.y * line2.x
				);
		}

		/**
		 * 判断点 P 是否在三角形ABC内
		 */
		static bool IsPointInTriangle(Vector3 A, Vector3 B, Vector3 C, Vector3 P, bool pointInSide = true)
		{
			return SameSide(A, B, C, P, pointInSide) &&
				SameSide(B, C, A, P, pointInSide) &&
				SameSide(C, A, B, P, pointInSide) ;
		}
		
		static bool GetSegmentLinesIntersection(const Vector3 p1, const Vector3 p2, const Vector3 p3, const Vector3 p4, Vector3& crossPoint)
		{
			const double z = p1.z;
			const auto denominator = (p2.y - p1.y) * (p4.x - p3.x) - (p1.x - p2.x) * (p3.y - p4.y);
			// 如果分母为0，则表示平行或共线，不相交
			if (denominator == 0)
			{
				return false;
			}
  
			// 线段所在直线的交点坐标 (x , y)
			const double x = ((p2.x - p1.x) * (p4.x - p3.x) * (p3.y - p1.y)
			  + (p2.y - p1.y) * (p4.x - p3.x) * p1.x
			  - (p4.y - p3.y) * (p2.x - p1.x) * p3.x) / denominator;
			const double y = -((p2.y - p1.y) * (p4.y - p3.y) * (p3.x - p1.x)
			  + (p2.x - p1.x) * (p4.y - p3.y) * p1.y
			  - (p4.x - p3.x) * (p2.y - p1.y) * p3.y) / denominator;
		
			// 判断交点是否在两条线段上
			if (
			  // 交点在线段1上
			  (x - p1.x) * (x - p2.x) <= 0 && (y - p1.y) * (y - p2.y) <= 0
			  // 且交点也在线段2上
			  && (x - p3.x) * (x - p4.x) <= 0 && (y - p3.y) * (y - p4.y) <= 0
			) {
				// 返回交点p
				crossPoint = Vector3(x, y, z);
				return true;
			}

			return false;
		}

		// 检查点是否在多边形内
		static bool IsPointInPolygonByRayCast(const std::vector<Vector3> quadPoints, const Vector3 p0)
		{
			const size_t pNum = quadPoints.size();
			int itJunctionCount = 0;
			for (int i = 0; i < pNum; i++) {
				const int ni = (i + 1) % pNum;
				const Vector3 pt_1 = quadPoints[i];
				const Vector3 pt_2 = quadPoints[ni];
				if (p0.y >= pt_1.y && p0.y <= pt_2.y || p0.y >= pt_2.y && p0.y <= pt_1.y) {
					const double duT = (p0.y - pt_1.y) / (pt_2.y - pt_1.y);
					const double duXT = pt_1.x + duT * (pt_2.x - pt_1.x);
				
					if (p0.x == duXT)
						return true;		// 在线段上表明为True
				
					if (p0.x > duXT)
						itJunctionCount++;
				}
			}
			return itJunctionCount % 2 == 1;
		}

		// 获取三角形的内心
		static Vector3 CalculateInsideCenter(Vector3 A, Vector3 B, Vector3 C)
		{
			const Vector3 AB = B - A;
			const Vector3 AC = C - A;
			const Vector3 BA = A - B;
			const Vector3 BC = C - B;
 
			Vector3 nBA = BA;
			nBA.normalise();
			Vector3 nBC = BC;
			nBC.normalise();

			const float radBAC = Acos(AB.dotProduct(AC) / (AB.length() * AC.length()));
			const float radABC = Acos(BA.dotProduct(BC) / (BA.length() * BC.length()));
 
			const float halfRadBAC = radBAC / 2.0f;
			const float halfRadABC = radABC / 2.0f;
 
			const float r2 = AB.length() / (Cos(halfRadBAC) * Sin(halfRadABC) / Sin(halfRadBAC) + Cos(halfRadABC));
 
			Vector3 P = ((nBA + nBC) / 2.0f).normalized() * r2 + B;
			return P;
		}

	private:
		static bool SameSide(Vector3 A, Vector3 B, Vector3 C, Vector3 P, bool isPointOnSide)
		{
			const Vector3 AB = B - A ;
			const Vector3 AC = C - A ;
			const Vector3 AP = P - A ;

			const Vector3 v1 = AB.crossProduct(AC) ;
			const Vector3 v2 = AB.crossProduct(AP) ;

			if (isPointOnSide)
				return v1.dotProduct(v2) >= 0 ;
			else
				return v1.dotProduct(v2) > 0 ;
		}

	};
}
