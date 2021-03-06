#pragma once

#include <vector>

#include "Vectors.h"

namespace NavMeshBase
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
		static double Dot(NavMeshBase::Vector3 line1, NavMeshBase::Vector3 line2)
		{
			return line1.x * line2.x + line1.y * line2.y;
		}

		// 叉乘
		static NavMeshBase::Vector3 Cross(NavMeshBase::Vector3 line1, NavMeshBase::Vector3 line2)
		{
			return NavMeshBase::Vector3(
				line1.y * line2.z - line1.z * line2.y,
				line1.z * line2.x - line1.x * line2.z,
				line1.x * line2.y - line1.y * line2.x
				);
		}

		/**
		 * 判断点 P 是否在三角形ABC内
		 */
		static bool IsPointInTriangle(NavMeshBase::Vector3 A, NavMeshBase::Vector3 B, NavMeshBase::Vector3 C, NavMeshBase::Vector3 P, bool pointInSide = true, double diff = 0)
		{
			return SameSide(A, B, C, P, pointInSide, diff) &&
				SameSide(B, C, A, P, pointInSide, diff) &&
				SameSide(C, A, B, P, pointInSide, diff) ;
		}
		
		static bool GetSegmentLinesIntersection(const NavMeshBase::Vector3 p1, const NavMeshBase::Vector3 p2, const NavMeshBase::Vector3 p3, const NavMeshBase::Vector3 p4, NavMeshBase::Vector3& crossPoint)
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
				crossPoint = NavMeshBase::Vector3(x, y, z);
				return true;
			}

			return false;
		}

		// 检查点是否在多边形内
		static bool IsPointInPolygonByRayCast(const std::vector<NavMeshBase::Vector3> quadPoints, const NavMeshBase::Vector3 p)
		{
			// 交点个数  
			int nCross = 0;
			const int nCount = quadPoints.size();
			for (int i = 0; i < nCount; i++)
			{
				Vector3 p1 = quadPoints[i];
				Vector3 p2 = quadPoints[(i + 1) % nCount];
 
				if (p1.y == p2.y)
					continue;
				if (p.y < std::min<double>(p1.y, p2.y))
					continue;
				if (p.y >= std::max<double>(p1.y, p2.y))
					continue;
				
				const double x = (p.y - p1.y) * (p2.x - p1.x) / (p2.y - p1.y) + p1.x;
				if (x > p.x)
					nCross++;
			}
			
			// 交点为偶数，点在多边形之外  
			return nCross % 2 == 1;
		}

		// 获取三角形的内心
		static NavMeshBase::Vector3 CalculateInsideCenter(NavMeshBase::Vector3 A, NavMeshBase::Vector3 B, NavMeshBase::Vector3 C)
		{
			const NavMeshBase::Vector3 AB = B - A;
			const NavMeshBase::Vector3 AC = C - A;
			const NavMeshBase::Vector3 BA = A - B;
			const NavMeshBase::Vector3 BC = C - B;
 
			NavMeshBase::Vector3 nBA = BA;
			nBA.normalise();
			NavMeshBase::Vector3 nBC = BC;
			nBC.normalise();

			const double radBAC = Acos(AB.dotProduct(AC) / (AB.length() * AC.length()));
			const double radABC = Acos(BA.dotProduct(BC) / (BA.length() * BC.length()));
 
			const double halfRadBAC = radBAC / 2.0f;
			const double halfRadABC = radABC / 2.0f;
 
			const double r2 = AB.length() / (Cos(halfRadBAC) * Sin(halfRadABC) / Sin(halfRadBAC) + Cos(halfRadABC));
 
			NavMeshBase::Vector3 P = ((nBA + nBC) / 2.0f).normalized() * r2 + B;
			return P;
		}

		// 计算点到线段的垂直距离
		static double GetDisFromPointToLine(NavMeshBase::Vector3 p, NavMeshBase::Vector3 p1, NavMeshBase::Vector3 p2)
		{
			const NavMeshBase::Vector3 lineVec = p2 - p1;
			if (lineVec.squaredLength() == 0)
				return (p - p1).length();
			
			NavMeshBase::Vector3 tp = (p - p1).absDotProduct(lineVec) / lineVec.length() * lineVec.normalisedCopy();
			return (p - p1 - tp).length();
		}

		// 获取点到线段垂直的点
		static NavMeshBase::Vector3 GetPosFromPointToLine(NavMeshBase::Vector3 p, NavMeshBase::Vector3 p1, NavMeshBase::Vector3 p2)
		{
			const NavMeshBase::Vector3 lineVec = p2 - p1;
			if (lineVec.squaredLength() == 0)
				return p1;
			
			return p1 + (p - p1).absDotProduct(lineVec) / lineVec.length() * lineVec;
		}

		/**
		 * 获取点到线段的最短距离
		 * shortPoint: 最短距离的点
		 */
		static double PointToSegDist(NavMeshBase::Vector3 p, NavMeshBase::Vector3 p1, NavMeshBase::Vector3 p2, NavMeshBase::Vector3& shortPoint)
		{
			double x = p.x, y = p.y, x1 = p1.x, y1 = p1.y, x2 = p2.x, y2 = p2.y;
			double cross = (x2 - x1) * (x - x1) + (y2 - y1) * (y - y1);
			
			if (cross <= 0)
			{
				shortPoint = p1;
				return sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1));
			}
  
			double d2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
			if (cross >= d2)
			{
				shortPoint = p2;
				return sqrt((x - x2) * (x - x2) + (y - y2) * (y - y2));
			}
  
			double r = cross / d2;
			double px = x1 + (x2 - x1) * r;
			double py = y1 + (y2 - y1) * r;
			
			shortPoint = NavMeshBase::Vector3(px, py, 0);
			return sqrt((x - px) * (x - px) + (py - y) * (py - y));
		}

	private:
		/**
		 *	different 检查误差
		 */
		static bool SameSide(NavMeshBase::Vector3 A, NavMeshBase::Vector3 B, NavMeshBase::Vector3 C, NavMeshBase::Vector3 P, bool isPointOnSide, double diff = 0)
		{
			const NavMeshBase::Vector3 AB = B - A ;
			const NavMeshBase::Vector3 AC = C - A ;
			const NavMeshBase::Vector3 AP = P - A ;

			const NavMeshBase::Vector3 v1 = AB.crossProduct(AC) ;
			const NavMeshBase::Vector3 v2 = AB.crossProduct(AP) ;

			if (isPointOnSide)
				return v1.dotProduct(v2) >= diff;
			else
				return v1.dotProduct(v2) > diff;
		}

	};
}
