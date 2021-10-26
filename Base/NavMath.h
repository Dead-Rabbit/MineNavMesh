#pragma once

#include "Graphes.h"

namespace ZXNavMesh
{
	class NavMath
	{
	public:
		NavMath(){};
		
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
		
	private:
		static bool SameSide(Vector3 A, Vector3 B, Vector3 C, Vector3 P)
		{
			Vector3 AB = B - A ;
			Vector3 AC = C - A ;
			Vector3 AP = P - A ;

			Vector3 v1 = AB.crossProduct(AC) ;
			Vector3 v2 = AB.crossProduct(AP) ;

			return v1.dotProduct(v2) >= 0 ;
		}

	};
}
