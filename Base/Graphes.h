#pragma once
#include "Vectors.h"

using namespace ZXNavMesh;

namespace ZXNavMesh
{
	class Graphes
	{
	public:
	};

	class Line
	{
	public:
		Line(Vector3 start, Vector3 end)
		{
			this->start = start;
			this->end = end;
		}
		
		Vector3 start;
		Vector3 end;
	};

	class Triangle
	{
	public:
		Vector3 A, B, C;
		Triangle(Vector3 A, Vector3 B, Vector3 C)
		{
			this->A = A;
			this->B = B;
			this->C = C; 
		}
	};
	
}