#pragma once
#include "Math.hpp"

struct Rect
{
	tako::Vector3 center;
	tako::Vector3 size;

	Rect()
	{
	}

	Rect(tako::Vector3 center, tako::Vector3 size): center(center), size(size)
	{
	}

	bool Contains(tako::Vector3 point)
	{
		return 
			point.x > center.x - size.x / 2 &&
			point.x < center.x + size.x / 2 &&
			point.y > center.y - size.y / 2 &&
			point.y < center.y + size.y / 2 &&
			point.z > center.z - size.z / 2 &&
			point.z < center.z + size.z / 2;
	}

	static bool Overlaps(Rect& a, Rect& b)
	{
		return 
			tako::mathf::abs(a.center.x - b.center.x) < a.size.x / 2 + b.size.x / 2 &&
			tako::mathf::abs(a.center.y - b.center.y) < a.size.y / 2 + b.size.y / 2 &&
			tako::mathf::abs(a.center.z - b.center.z) < a.size.z / 2 + b.size.z / 2;
	}
};