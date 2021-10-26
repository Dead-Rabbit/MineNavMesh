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

		// 点是否在三角形内
		static bool IsInTrigon(Vector3 _target, Vector3 _center, Vector3 _left, Vector3 _right){
			Vector3 Ctl= _left - _center;
			Vector3 Ctr= _right - _center;
			Vector3 Ctt= _target - _center;
			Vector3 Ltr= _right - _left;
			Vector3 Ltc= _right - _center;
			Vector3 Ltt= _left - _target;
			Vector3 Rtl= _left - _right;
			Vector3 Rtc= _center - _right;
			Vector3 Rtt= _target - _right;
			if(
				Ctl.crossProduct(Ctr).normalized().dotProduct(Ctl.crossProduct(Ctt).normalized())==1&&
				Ltr.crossProduct(Ltc).normalized().dotProduct(Ltr.crossProduct(Ltt).normalized())==1&&
				Rtc.crossProduct(Rtl).normalized().dotProduct(Rtc.crossProduct(Rtt).normalized())==1
			)
				return true;
			
			return false;
		}
	};
}
