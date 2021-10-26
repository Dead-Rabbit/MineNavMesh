#pragma once

#include "Graphes.h"

namespace ZXNavMesh
{
	class NavMath
	{
	public:
		NavMath(){};

		static double Abs(double value)
		{
			if (value < 0)
				return -value;
			
			return value;
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
		static bool IsPointInTriangle(Vector3 A, Vector3 B, Vector3 C, Vector3 P)
		{
			return SameSide(A, B, C, P) &&
				SameSide(B, C, A, P) &&
				SameSide(C, A, B, P) ;
		}
		
		static bool GetSegmentLinesIntersection(const Vector3 p1, const Vector3 p2, const Vector3 p3, const Vector3 p4, Vector3& crossPoint)
		{
			double z = p1.z;
			auto denominator = (p2.y - p1.y) * (p4.x - p3.x) - (p1.x - p2.x) * (p3.y - p4.y);
			// 如果分母为0，则表示平行或共线，不相交
			if (denominator == 0)
			{
				return false;
			}
  
			// 线段所在直线的交点坐标 (x , y)
			double x = ((p2.x - p1.x) * (p4.x - p3.x) * (p3.y - p1.y)
			  + (p2.y - p1.y) * (p4.x - p3.x) * p1.x
			  - (p4.y - p3.y) * (p2.x - p1.x) * p3.x) / denominator;
			double y = -((p2.y - p1.y) * (p4.y - p3.y) * (p3.x - p1.x)
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
		
	private:
		static bool SameSide(Vector3 A, Vector3 B, Vector3 C, Vector3 P)
		{
			Vector3 AB = B - A ;
			Vector3 AC = C - A ;
			Vector3 AP = P - A ;

			Vector3 v1 = AB.crossProduct(AC) ;
			Vector3 v2 = AB.crossProduct(AP) ;

			return v1.dotProduct(v2) > 0 ;
		}

	};
}
