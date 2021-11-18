#include "NavMath.h"
#include "Vectors.h"

using namespace NavMeshBase;

int main(int argc, char* argv[])
{
	Vector3 p1 = Vector3(0, 0, 0);
	Vector3 p2 = Vector3(1, 1, 0);
	Vector3 p = Vector3(-0.5, 0.25, 0);
	Vector3 shortPoint = Vector3();
	double length = NavMath::PointToSegDist(p, p1, p2, shortPoint);

	std::cout << "Length: " << length << " - Point: " << shortPoint << std::endl;
	
	return 0;
}
