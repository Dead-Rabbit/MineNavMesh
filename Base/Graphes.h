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
	
}