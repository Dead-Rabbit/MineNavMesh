/*******************************************************************************
* Author    :  Angus Johnson                                                   *
* Version   :  10.0 (beta)                                                     *
* Date      :  21 November 2020                                                *
* Website   :  http://www.angusj.com                                           *
* Copyright :  Angus Johnson 2010-2020                                         *
* Purpose   :  Core Clipper Library structures and functions                   *
* License   :  http://www.boost.org/LICENSE_1_0.txt                            *
*                                                                              *
* C++       :  Thanks to help from Andreas Lücke - ALuecke@gmx.net             *
*******************************************************************************/

#ifndef CLIPPER_CORE_H
#define CLIPPER_CORE_H

#include "stdint.h"
#include <stdlib.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <iostream>
#include <limits>
#include <ostream>
#include <stdexcept>
#include <vector>
#include <type_traits>

namespace clipperlib {

const double floating_point_tolerance = 1E-15;           //floating point tolerance for equality
const double default_min_edge_len = 0.2;  //minimum edge length for stripping duplicates
const double sqrt_two = 1.4142135623731;
const double one_degree_as_radians = 0.01745329252;

// Point -----------------------------------------------------------------------

template <typename T>
struct Point;

using PointI = Point<int64_t>;
using PointD = Point<double>;

template <typename T>
struct Point {
	T x;
	T y;

	Point(T x = 0, T y = 0) :
		x(x), 
    y(y){};

	template <typename T2>
	explicit Point<T>(Point<T2> p) : x(static_cast<T>(p.x)), y(static_cast<T>(p.y))
	{};

	void Rotate(const PointD &center, double angle_rad)
	{
	    double tmp_x = x - center.x;
	    double tmp_y = y - center.y;
	    double cos_a = cos(angle_rad);
	    double sin_a = sin(angle_rad);

	    if  (std::numeric_limits<T>::is_integer) {
	        x = static_cast<T>(round(tmp_x * cos_a - tmp_y * sin_a + center.x));
	        y = static_cast<T>(round(tmp_x * sin_a - tmp_y * cos_a + center.y));
	    }
	    else {
	        x = static_cast<T>(tmp_x * cos_a - tmp_y * sin_a + center.x);
	        y = static_cast<T>(tmp_x * sin_a - tmp_y * cos_a + center.y);
	    }
	}

	void Rotate(const PointD &center, double sin_a, double cos_a)
	{
		double tmp_x = x - center.x;
		double tmp_y = y - center.y;

		if  (std::numeric_limits<T>::is_integer) {
			x = static_cast<T>(round(tmp_x * cos_a - tmp_y * sin_a + center.x));
			y = static_cast<T>(round(tmp_x * sin_a - tmp_y * cos_a + center.y));
		}
		else {
			x = static_cast<T>(tmp_x * cos_a - tmp_y * sin_a + center.x);
			y = static_cast<T>(tmp_x * sin_a - tmp_y * cos_a + center.y);
		}
	}

	friend inline bool operator==(const Point &a, const Point &b) {
		return a.x == b.x && a.y == b.y;
	}

	inline Point<T> operator-() const
	{
		return Point<T>(-x,-y);
	}

	inline PointD operator+(const PointD &b) const
	{
		return PointD(x+b.x, y+b.y);
	}
	inline PointD operator-(const PointD &b) const
	{
		return PointD(x-b.x, y-b.y);
	}

	inline PointD operator*(const double factor) const
	{
		return PointD(x*factor, y*factor);
	}

	friend inline bool operator!=(const Point &a, const Point &b) {
		return !(a == b);
	}
	friend inline bool operator<(const Point &a, const Point &b) {
		return (a.x == b.x) ? (a.y < b.y) : (a.x < b.x);
	}
	friend std::ostream &operator<<(std::ostream &os, const Point<T> &point) {
		os << "(" << point.x << "," << point.y << ")";
		return os;
	}
};

template <typename T>
PointI Round(Point<T> p)
{
	return PointI(static_cast<int64_t>(round(p.x)), static_cast<int64_t>(round(p.y)));
}

template <typename T>
inline bool NearEqual(const Point<T> p1, const Point<T> p2, double min_dist_sqrd) {
	return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) < min_dist_sqrd;
}


// Rect ------------------------------------------------------------------------

template <typename T>
struct Rect;

using RectI = Rect<int64_t>;
using RectD = Rect<double>;

template <typename T>
struct Rect {
	T left;
	T top;
	T right;
	T bottom;

	Rect() :
		left(0),
		top(0),
		right(0),
		bottom(0) {}

	Rect(T l, T t, T r, T b) :
		left(l),
		top(t),
		right(r),
		bottom(b) {}


	T Width() const { return right - left; }
	T Height() const { return bottom - top; }
	void Width(T _width) { right = left + _width; }
	void Height(T _height) { bottom = top + _height; }

	bool IsEmpty() const { return bottom <= top || right <= left; };

	void Inflate(T dx, T dy) {
		left -= dx;
		right += dx;
		top -= dy;
		bottom += dy;
	}

	void Offset(T dx, T dy) {
		left += dx;
		right += dx;
		top += dy;
		bottom += dy;
	}

	void Intersect(const Rect<T> &rect)
	{
		if (IsEmpty())
			return;
		else if (rect.IsEmpty()) {
			*this = Rect();
		} else {
			left = std::max(rect.left, left);
			right = std::min(rect.right, right);
			top = std::max(rect.top, top);
			bottom = std::min(rect.bottom, bottom);
			if (IsEmpty())
				*this = Rect();
		}
	}
	void Union(const Rect<T> &rect){
		if (rect.IsEmpty())
			return;
		else if (IsEmpty()) {
			*this = rect;
			return;
		}
		left = std::min(rect.left, left);
		right = std::max(rect.right, right);
		top = std::min(rect.top, top);
		bottom = std::max(rect.bottom, bottom);
	}

	void Rotate(double angle_rad); //Needs path declaration first

	void Scale(double scale) { left *= scale; top *= scale; right *= scale; bottom *= scale; };

	friend std::ostream &operator<<(std::ostream &os, const Rect<T> &rect) {
		os << "("
		   << rect.left << "," << rect.top << "," << rect.right << "," << rect.bottom
		   << ")";
		return os;
	}
};

// ClipperLibException ---------------------------------------------------------

class ClipperLibException : public std::exception {
public:
	explicit ClipperLibException(const char *description) :
		m_descr(description) {}
	virtual const char *what() const throw() { return m_descr.c_str(); }

private:
	std::string m_descr;
};


// Path ------------------------------------------------------------------------

//Path: a simple data structure to represent a series of vertices, whether
//open (poly-line) or closed (polygon). A path may be simple or complex (self
//intersecting). For simple polygons, path orientation (whether clockwise or
//counter-clockwise) is generally used to differentiate outer paths from inner
//paths (holes). For complex polygons (and also for overlapping polygons),
//explicit 'filling rules' (see below) are used to indicate regions that are
//inside (filled) and regions that are outside (unfilled) a specific polygon.

template <typename T>
struct Path;
using PathI = Path<int64_t>;
using PathD = Path<double>;

template<typename T>
struct Path {
	std::vector<Point<T> > data;

	using Size = decltype(data.size());
	Size size() const { return data.size(); }
	void resize(Size size) { data.resize(size); }
	bool empty() const { return data.size() == 0; }
	void reserve(Size size) { data.reserve(size); }
	void push_back(const Point<T>& point) { data.push_back(point); }
	void pop_back() { data.pop_back(); }
	void clear() { data.clear(); }

	Path() {}

	Point<T> &operator[](Size idx) { return data[idx]; }
	const Point<T> &operator[](Size idx) const { return data[idx]; }

	void Append(const Path<T> &extra) {
	  if (extra.size() > 0)
	    data.insert(end(data), begin(extra.data), end(extra.data));
	}

	double Area() const {
		double area = 0.0;
		auto len = data.size() - 1;
		if (len < 2) return area;
		auto j = len;
		for (decltype(len) i = 0; i <= len; ++i) {
			double d = static_cast<double>(data[j].x + data[i].x);
			area += d * (data[j].y - data[i].y);
			j = i;
		}
		return -area * 0.5;
	}

	Rect<T> Bounds() const {
		const T _MAX = std::numeric_limits<T>::max();
		const T _MIN = std::numeric_limits<T>::lowest(); //   -_MAX;

		Rect<T> bounds(_MAX, _MAX, _MIN, _MIN);

		for (const auto &point : data) {
			if (point.x < bounds.left) bounds.left = point.x;
			if (point.x > bounds.right) bounds.right = point.x;
			if (point.y < bounds.top) bounds.top = point.y;
			if (point.y > bounds.bottom) bounds.bottom = point.y;
		}

		if (bounds.left >= bounds.right)
			return Rect<T>();
		else
			return bounds;
	}
	void Offset(T dx, T dy){
		if (dx == 0 && dy == 0) return;
		for (auto &point : data) {
			point.x += dx;
			point.y += dy;
		}
	}
	bool Orientation() const {
		return Area() >= 0;
	}
	void Reverse() {
		std::reverse(begin(data), end(data));
	}
	void Rotate(const PointD &center, double angle_rad) {
		double cos_a = cos(angle_rad);
		double sin_a = sin(angle_rad);

		for (auto &point : data)
			point.Rotate(center, sin_a, cos_a);
	}
	void Scale(double sx, double sy){
		if (sx == 0) sx = 1;
		if (sy == 0) sy = 1;
		if (sx == 1 && sy == 1) return;

		if  (std::numeric_limits<T>::is_integer)
		{
			for (auto& point : data) {
				point.x = static_cast<T>(round(point.x * sx));
				point.y = static_cast<T>(round(point.y * sy));
			}
		}
		else
		{
			for (auto& point : data) {
				point.x = static_cast<T>(point.x * sx);
				point.y = static_cast<T>(point.y * sy);
			}
		}
		StripDuplicates();
	}
	void StripDuplicates(bool is_closed_path = false, T min_length = 0){
		if (std::numeric_limits<T>::is_integer && min_length < 1)
		{
			data.erase(unique(begin(data), end(data)), end(data));
		}
		else
		{
			if (data.size() < 2) return;
			if (min_length < floating_point_tolerance)
				min_length = default_min_edge_len;
			for (auto it = data.begin() + 1; it != data.end(); )
				if (NearEqual(*(it - 1), *it, min_length * min_length))
					it = data.erase(it);
				else
					++it;
		}
		size_t len = data.size();
		if (!is_closed_path || len == 0) return;
		if (NearEqual(data[0], data[len - 1], min_length * min_length))
			data.resize(len - 1);
	}
	//void Trim(bool is_closed_path, T min_length);

	template<typename T2>		
	void AppendPointsScale(const Path<T2> & other, double scale)
	{
		data.reserve(data.size() + other.size());
		if  (std::numeric_limits<T>::is_integer)
		{
			std::transform(other.data.begin(),other.data.end(),std::back_inserter(data),[scale](Point<T2> p) {return Point<T>(Round(p*scale));});
			//for (const auto &p : other.data)
			//	data.push_back(Point<T>(Round(p*scale)));
		}
		else
		{
			std::transform(other.data.begin(),other.data.end(),std::back_inserter(data),[scale](Point<T2> p) {return Point<T>(p*scale);});
			//	for (const auto &p : other.data)
			//		data.push_back(Point<T>(p * scale));
		}
	}

	Path(const Path<T> & other, double scale){
		if (scale == 0) scale = 1;
		if (scale == 1) {
			Append(other);
		} else {
			AppendPointsScale(other,scale);
		}
	}

	template<typename T2, typename=
	typename std::enable_if<!std::is_same<T, T2>::value,T>::type >
	Path(const Path<T2> & other, double scale){
		if (scale == 0) scale = 1;
		AppendPointsScale(other,scale);
	}

	template<typename T2, typename=
	typename std::enable_if<!std::is_same<T, T2>::value,T>::type >
	void Assign(const Path<T2> & other, double scale){
		if (&other == reinterpret_cast<Path<T2>*>(this))
		    throw ClipperLibException("Can't assign self to self in Path<T>::Assign.");
		data.clear();
		if (scale == 0) scale = 1;
		AppendPointsScale(other, scale);
	}

	void Assign(const Path<T> & other, double scale){
		if (&other == reinterpret_cast<Path<T>*>(this))
		    throw ClipperLibException("Can't assign self to self in Path<T>::Assign.");
		data.clear();
		if (scale == 0) scale = 1;
		if (scale == 1) {
			Append(other);
		} else {
			AppendPointsScale(other, scale);
		}

	}

	friend inline Path<T> &operator<<(Path<T> &path, const Point<T> &point) {
		path.data.push_back(point);
		return path;
	}
	friend std::ostream &operator<<(std::ostream &os, const Path<T> &path) {
		if (path.data.empty())
			return os;

		Size last = path.size() - 1;

		for (Size i = 0; i < last; ++i)
			os << "(" << path[i].x << "," << path[i].y << "), ";

		os << "(" << path[last].x << "," << path[last].y << ")\n";

		return os;
	}
};

// Paths -----------------------------------------------------------------------

template <typename T>
struct Paths;
using PathsI = Paths<int64_t>;
using PathsD = Paths<double>;

template <typename T>
struct Paths {
	std::vector<Path<T>> data;

	using Size = decltype(data.size());
	Size size() const { return data.size(); }
	void resize(Size size) { data.resize(size); }
	void reserve(Size size) { data.reserve(size); }
	void push_back(const Path<T> &path) { data.push_back(path); }
	void clear() { data.clear(); }

	Path<T> &operator[](Size idx) { return data[idx]; }
	const Path<T> &operator[](Size idx) const { return data[idx]; }

	Paths() {}
	Paths(const PathsI &other, double scale = 1.0) {} //Specialization for PathsI and PathsD after the class declaration
	Paths(const PathsD &other, double scale = 1.0) {} //Specialization for PathsI and PathsD after the class declaration

	void Append(const Paths<T> &extra){
		if (extra.size() > 0)
		    data.insert(end(data), begin(extra.data), end(extra.data));
	}
	void Assign(const PathsI &other, double scale = 1.0);  //Specialization for PathsI and PathsD after the class declaration
	void Assign(const PathsD &other, double scale = 1.0);  //Specialization for PathsI and PathsD after the class declaration
	Rect<T> Bounds() const {
		const T _MAX = std::numeric_limits<T>::max();
		const T _MIN = std::numeric_limits<T>::lowest(); //-_MAX;

		Rect<T> bounds(_MAX, _MAX, _MIN, _MIN);

		for (const auto &path : data) {
			for (const auto &point : path.data) {
				if (point.x < bounds.left) bounds.left = point.x;
				if (point.x > bounds.right) bounds.right = point.x;
				if (point.y < bounds.top) bounds.top = point.y;
				if (point.y > bounds.bottom) bounds.bottom = point.y;
			}
		}

		if (bounds.left >= bounds.right)
			return Rect<T>();
		else
			return bounds;
	}
	void Offset(T dx, T dy) {
		if (dx == 0 && dy == 0) return;
		for (auto &path : data)
			for (auto &point : path.data) {
				point.x += dx;
				point.y += dy;
			}
	}
	void Reverse(){
		for (auto &path : data)
			path.Reverse();
	}
	void Rotate(const PointD &center, double angle_rad){
		double cos_a = cos(angle_rad);
		double sin_a = sin(angle_rad);

		for (auto &path : data)
			for (auto &point : path.data)
				point.Rotate(center, sin_a, cos_a);
	}
	void Scale(double scale_x, double scale_y){
		for (auto &path : data)
			path.Scale(scale_x, scale_y);
	}
	void StripDuplicates(bool is_closed_path, T min_length){
		for (auto& path : data)
			path.StripDuplicates(is_closed_path, min_length);
	}

	template<typename T2>
	void AppendPointsScale(const Paths<T2>& other, double scale) {
		size_t other_size = other.size();
		data.resize(other_size);
		for (size_t i = 0; i < other_size; ++i)			
			data[i].AppendPointsScale(other[i], scale);

	}

	friend inline Paths<T> &operator<<(Paths<T> &paths, const Path<T> &path) {
		paths.data.push_back(path);
		return paths;
	}

	friend std::ostream &operator<<(std::ostream &os, const Paths<T> &paths) {
		for (Size i = 0; i < paths.size(); ++i)
			os << paths[i];
		os << "\n";
		return os;
	}
};

//------------------------------------------------------------------------------
// Specialization functions for Paths
//------------------------------------------------------------------------------

template<>
inline void PathsI::Assign(const PathsI &other, double scale) {
	data.clear();
	data.resize(other.data.size());
	typename std::vector<PathI>::iterator it1;
	typename std::vector<PathI>::const_iterator it2;
	for (it1 = data.begin(), it2 = other.data.begin(); it1 != data.end(); ++it1, ++it2)
		it1->Assign(*it2, scale);
}
//------------------------------------------------------------------------------

template<>
inline void PathsD::Assign(const PathsI &other, double scale) {
	data.clear();
	data.resize(other.data.size());
	typename std::vector<PathD>::iterator it1;
	typename std::vector<PathI>::const_iterator it2;
	for (it1 = data.begin(), it2 = other.data.begin(); it1 != data.end(); ++it1, ++it2)
		it1->Assign(*it2, scale);
}
//------------------------------------------------------------------------------
template<>
inline void PathsI::Assign(const PathsD &other, double scale) {
	data.clear();
	data.resize(other.data.size());
	typename std::vector<PathI>::iterator it1;
	typename std::vector<PathD>::const_iterator it2;
	for (it1 = data.begin(), it2 = other.data.begin(); it1 != data.end(); ++it1, ++it2)
		it1->Assign(*it2, scale);
}
//------------------------------------------------------------------------------

template <>
inline void PathsD::Assign(const PathsD &other, double scale) {
	data.clear();
	data.resize(other.data.size());
	typename std::vector<PathD>::iterator it1;
	typename std::vector<PathD>::const_iterator it2;
	for (it1 = data.begin(), it2 = other.data.begin(); it1 != data.end(); ++it1, ++it2)
		it1->Assign(*it2, scale);
}
//------------------------------------------------------------------------------

template<typename T>
void clipperlib::Paths<T>::Assign(const PathsI & other, double scale){}
//------------------------------------------------------------------------------

template<typename T>
void clipperlib::Paths<T>::Assign(const PathsD & other, double scale){}
//------------------------------------------------------------------------------

template <>
inline PathsI::Paths(const PathsI &other, double scale) {
  Assign(other, scale);
}
//------------------------------------------------------------------------------

template <>
inline PathsD::Paths(const PathsI &other, double scale) {
  Assign(other, scale);
}
//------------------------------------------------------------------------------

template <>
inline PathsI::Paths(const PathsD &other, double scale) {
  Assign(other, scale);
}
//------------------------------------------------------------------------------

template <>
inline PathsD::Paths(const PathsD &other, double scale) {
  Assign(other, scale);
}
//------------------------------------------------------------------------------


// PathsArray ------------------------------------------------------------------

template <typename T>
struct PathsArray {
	std::vector<Paths<T> > data;

	using Size = decltype(data.size());

	Size size() const { return data.size(); }
	void resize(Size size) { data.resize(size); }
	void reserve(Size size) { data.reserve(size); }
	void push_back(const Paths<T> &paths) { data.push_back(paths); }
	void clear() { data.clear(); }

	Paths<T> &operator[](Size idx) { return data[idx]; }
	const Paths<T> &operator[](Size idx) const { return data[idx]; }

	Rect<T> Bounds() const{
		const T _MAX = std::numeric_limits<T>::max();
		const T _MIN = std::numeric_limits<T>::lowest(); //-_MAX;

		Rect<T> bounds(_MAX, _MAX, _MIN, _MIN);

		for (const auto &paths : data) {
			for (const auto &path : paths.data) {
				for (const auto &point : path.data) {
					if (point.x < bounds.left) bounds.left = point.x;
					if (point.x > bounds.right) bounds.right = point.x;
					if (point.y < bounds.top) bounds.top = point.y;
					if (point.y > bounds.bottom) bounds.bottom = point.y;
				}
			}
		}

		if (bounds.left >= bounds.right)
			return Rect<T>();
		else
			return bounds;
	}
};

using PathsArrayI = PathsArray<int64_t>;
using PathsArrayD = PathsArray<double>;


//Rect function Rotate needs declaration of path first
template <typename T>
inline void Rect<T>::Rotate(double angle_rad) {
		using UsedT = typename std::conditional<std::numeric_limits<T>::is_integer, double, T>::type;
		Point<UsedT> cp;
		cp.x = static_cast<UsedT>((right + left) / 2);
		cp.y = static_cast<UsedT>((bottom + top) / 2);

		Path<UsedT> pts;
		pts.resize(4);
		pts[0] = Point<UsedT>(static_cast<UsedT>(left), static_cast<UsedT>(top));
		pts[1] = Point<UsedT>(static_cast<UsedT>(right), static_cast<UsedT>(top));
		pts[2] = Point<UsedT>(static_cast<UsedT>(right), static_cast<UsedT>(bottom));
		pts[3] = Point<UsedT>(static_cast<UsedT>(left), static_cast<UsedT>(bottom));

		pts.Rotate(cp, angle_rad);

		const auto resultx = std::minmax_element(begin(pts.data), end(pts.data),[](Point<UsedT> p1, Point<UsedT> p2) {return p1.x< p2.x;});
		const auto resulty = std::minmax_element(begin(pts.data), end(pts.data),[](Point<UsedT> p1, Point<UsedT> p2) {return p1.y< p2.y;});

		if  (std::numeric_limits<T>::is_integer) {
			left = static_cast<T>(std::floor(resultx.first->x));
			right = static_cast<T>(std::ceil(resultx.second->x));
			top = static_cast<T>(std::floor(resulty.first->y));
			bottom = static_cast<T>(std::ceil(resulty.second->y));
		}
		else
		{
			left = static_cast<T>(resultx.first->x);
			right = static_cast<T>(resultx.second->x);
			top = static_cast<T>(resulty.first->y);
			bottom = static_cast<T>(resulty.second->y);
		}
	}

// Miscellaneous ---------------------------------------------------------------

template <typename T>
T CrossProduct(const Point<T>& pt1, const Point<T>& pt2, const Point<T>& pt3) {
	return ((pt2.x - pt1.x) * (pt3.y - pt2.y) - (pt2.y - pt1.y) * (pt3.x - pt2.x));
}
//------------------------------------------------------------------------------

template <typename T>
double DistanceSqr(const Point<T> pt1, const Point<T> pt2) {
	return std::pow(pt1.x - pt2.x, 2.0) + std::pow(pt1.y - pt2.y, 2.0);
}
//------------------------------------------------------------------------------

template <typename T>
double DistanceFromLineSqrd(const Point<T> &pt, const Point<T> &ln1, const Point<T> &ln2)
{
	//perpendicular distance of point (x³,y³) = (Ax³ + By³ + C)/Sqrt(A² + B²)
	//see http://en.wikipedia.org/wiki/Perpendicular_distance
	double A = (ln1.y - ln2.y);
	double B = (ln2.x - ln1.x);
	double C = A * ln1.x + B * ln1.y;
	C = A * pt.x + B * pt.y - C;
	return (C * C) / (A * A + B * B);
}
//---------------------------------------------------------------------------

template <typename T>
bool NearCollinear(const Point<T> &pt1, const Point<T> &pt2, const Point<T> &pt3, double sin_sqrd_min_angle_rads)
{
	double cp = std::abs(CrossProduct(pt1, pt2, pt3));
	return (cp * cp) / (DistanceSqr(pt1, pt2) * DistanceSqr(pt2, pt3)) < sin_sqrd_min_angle_rads;
}
//------------------------------------------------------------------------------

template <typename T>
void CleanPathWithSinAngleRads(Path<T>& path, bool is_closed, double min_length, double sin_min_angle_in_radians)
{
	if (path.size() < 2) return;
	//clean up insignificant edges
	double distSqrd = min_length * min_length;
	typename std::vector<Point<T>>::iterator it;
	for (it = path.data.begin() + 1; it != path.data.end(); )
	{
		if (NearEqual(*(it -1), *it, distSqrd))
			it = path.data.erase(it);
		else
			++it;
	}
	size_t len = path.size();
	if (is_closed && NearEqual(path[0], path[len - 1], distSqrd)) path.pop_back();

	if (path.size() < 3) return;
	double sin_sqrd_min_angle = sin(sin_min_angle_in_radians);
	sin_sqrd_min_angle *= sin_sqrd_min_angle;

	//clean up near colinear edges
	for (it = path.data.begin() + 2; it != path.data.end(); ++it)
		if (NearCollinear(*(it - 2), *(it - 1), *it, sin_sqrd_min_angle))
			it = path.data.erase(it - 1);

	len = path.size();
	if (len > 2 && is_closed && 
		NearCollinear(path[len - 2], path[len - 1], path[0], sin_sqrd_min_angle))
			path.pop_back();
}
//------------------------------------------------------------------------------

template <typename T>
void CleanPath(Path<T> &path, bool is_closed, double min_length, double min_angle_in_radians)
{
	CleanPathWithSinAngleRads(path, is_closed, min_length, std::sin(min_angle_in_radians));
}
//------------------------------------------------------------------------------

template <typename T>
void CleanPaths(Paths<T> &paths, bool is_closed, double min_length, double min_angle_in_radians)
{
	double sine = std::sin(min_angle_in_radians);
	typename std::vector<Path<T>>::iterator it;
	for (it = paths.data.begin(); it != paths.data.end(); ++it)
		CleanPathWithSinAngleRads(*it, is_closed, min_length, sine);
}
//------------------------------------------------------------------------------

//Note: all clipping operations except for Difference are commutative.
enum class ClipType { None, Intersection, Union, Difference, Xor };

enum class PathType { Subject, Clip };

//By far the most widely used filling rules for polygons are EvenOdd
//and NonZero, sometimes called Alternate and Winding respectively.
//https://en.wikipedia.org/wiki/Nonzero-rule
enum class FillRule { EvenOdd, NonZero, Positive, Negative };

//PointInPolygon
enum class PipResult { Inside, Outside, OnEdge };

PipResult PointInPolygon(const PointI &pt, const PathI &path);


}  // namespace clipperlib

#endif  // CLIPPER_CORE_H
