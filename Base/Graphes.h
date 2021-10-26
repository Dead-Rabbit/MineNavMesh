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
		Line(Vector2 start, Vector2 end)
		{
			this->start = start;
			this->end = end;
		}
		
		Vector2 start;
		Vector2 end;
	};
	
}