#ifndef _Vectors_h_
#define _Vectors_h_

#include <iostream>
#include <cassert>
#include <easyx.h>

namespace NavMeshBase
{
	class Vector2
	{
	public:
		double x, y;

		inline Vector2() { }
		inline Vector2(const double fX, const double fY) : x(fX), y(fY) { }
		inline explicit Vector2(const double afCoordinate[2]) : x(afCoordinate[0]), y(afCoordinate[1]) { }

		inline Vector2& operator = (const Vector2& rkVector)
		{
			x = rkVector.x;
			y = rkVector.y;
			return *this;
		}

		inline bool operator == (const Vector2& rkVector) const { return (x == rkVector.x && y == rkVector.y); }
		inline bool operator != (const Vector2& rkVector) const { return (x != rkVector.x || y != rkVector.y); }
		inline Vector2 operator + (const Vector2& rkVector) const { return Vector2(x + rkVector.x, y + rkVector.y); }
		inline Vector2 operator - (const Vector2& rkVector) const { return Vector2(x - rkVector.x, y - rkVector.y); }
		inline Vector2 operator * (const double fScalar) const { return Vector2(x * fScalar, y * fScalar); }
		inline Vector2 operator / (const double fScalar) const { return Vector2(x / fScalar, y / fScalar); }
		inline const Vector2& operator + () const { return *this; }
		inline Vector2 operator - () const { return Vector2(-x, -y); }
		inline friend Vector2 operator * (const double fScalar, const Vector2& rkVector) { return Vector2(fScalar * rkVector.x, fScalar * rkVector.y); }

		inline Vector2& operator += (const Vector2& rkVector)
		{
			x += rkVector.x;
			y += rkVector.y;
			return *this;
		}

		inline Vector2& operator -= (const Vector2& rkVector)
		{
			x -= rkVector.x;
			y -= rkVector.y;
			return *this;
		}

		inline Vector2& operator *= (const double fScalar)
		{
			x *= fScalar;
			y *= fScalar;
			return *this;
		}

		inline Vector2& operator /= (const double fScalar)
		{
			assert(fScalar != 0.0);
			double fInv = 1.0f / fScalar;
			x *= fInv;
			y *= fInv;
			return *this;
		}

		inline double length() const { return std::sqrt(x * x + y * y); }
		inline double squaredLength() const { return x * x + y * y; }
		inline double dotProduct(const Vector2& vec) const { return x * vec.x + y * vec.y; }
		inline double crossProduct(const Vector2& rkVector) const { return x * rkVector.y - y * rkVector.x; }

		inline double normalise()
		{
			double fLength = std::sqrt(x * x + y * y);
			if (fLength > double(0.0f))
			{
				double fInvLength = 1.0f / fLength;
				x *= fInvLength;
				y *= fInvLength;
			}
			return fLength;
		}
	};

	class Vector3
    {
    public:
		double x, y, z;

    public:
        inline Vector3() { }
        inline Vector3( const double fX, const double fY, const double fZ ) : x( fX ), y( fY ), z( fZ ) { }
        inline explicit Vector3( const double afCoordinate[3] ) : x( afCoordinate[0] ), y( afCoordinate[1] ), z( afCoordinate[2] ) { }

		inline double operator [] ( const size_t i ) const { assert( i < 3 ); return *(&x+i); }
		inline double& operator [] ( const size_t i ) { assert( i < 3 ); return *(&x+i); }

		inline double* ptr() { return &x; }
		inline const double* ptr() const { return &x; }

        inline Vector3& operator = ( const Vector3& rkVector )
        {
            x = rkVector.x;
            y = rkVector.y;
            z = rkVector.z;
            return *this;
        }

        inline bool operator == ( const Vector3& rkVector ) const { return ( x == rkVector.x && y == rkVector.y && z == rkVector.z ); }
        inline bool operator != ( const Vector3& rkVector ) const { return ( x != rkVector.x || y != rkVector.y || z != rkVector.z ); }
        inline Vector3 operator + ( const Vector3& rkVector ) const { return Vector3(x + rkVector.x, y + rkVector.y, z + rkVector.z); }
        inline Vector3 operator - ( const Vector3& rkVector ) const { return Vector3(x - rkVector.x, y - rkVector.y, z - rkVector.z); }
        inline Vector3 operator * ( const double fScalar ) const { return Vector3(x * fScalar, y * fScalar, z * fScalar); }
        inline Vector3 operator / ( const double fScalar ) const { return Vector3(x / fScalar, y / fScalar, z / fScalar);  }
        inline const Vector3& operator + () const { return *this; }
        inline Vector3 operator - () const { return Vector3(-x, -y, -z); }
        inline friend Vector3 operator * ( const double fScalar, const Vector3& rkVector ) { return Vector3(fScalar * rkVector.x, fScalar * rkVector.y, fScalar * rkVector.z); }

        inline Vector3& operator += ( const Vector3& rkVector )
        {
            x += rkVector.x;
            y += rkVector.y;
            z += rkVector.z;
            return *this;
        }

        inline Vector3& operator -= ( const Vector3& rkVector )
        {
            x -= rkVector.x;
            y -= rkVector.y;
            z -= rkVector.z;
            return *this;
        }

        inline Vector3& operator *= ( const double fScalar )
        {
            x *= fScalar;
            y *= fScalar;
            z *= fScalar;
            return *this;
        }

        inline Vector3& operator /= ( const double fScalar )
        {
            assert( fScalar != 0.0 );
            double fInv = 1.0f / fScalar;
            x *= fInv;
            y *= fInv;
            z *= fInv;
            return *this;
        }
		
		friend std::ostream &operator<<(std::ostream &os, const Vector3 point) {
        	os << "(" << point.x << "," << point.y << ")";
        	return os;
        }

        inline double length () const { return std::sqrt( x * x + y * y + z * z ); }
        inline double squaredLength () const { return x * x + y * y + z * z; }
        inline double dotProduct(const Vector3& vec) const { return x * vec.x + y * vec.y + z * vec.z; }
        inline double absDotProduct(const Vector3& vec) const { return std::abs(x * vec.x) + std::abs(y * vec.y) + std::abs(z * vec.z); }
		inline Vector3 crossProduct(const Vector3& rkVector) const { return Vector3(y * rkVector.z - z * rkVector.y, z * rkVector.x - x * rkVector.z, x * rkVector.y - y * rkVector.x); }
        inline Vector3 midPoint( const Vector3& vec ) const { return Vector3( ( x + vec.x ) * 0.5f, ( y + vec.y ) * 0.5f, ( z + vec.z ) * 0.5f ); }

        inline double normalise()
        {
            double fLength = std::sqrt( x * x + y * y + z * z );
            if ( fLength > double(0.0f) )
            {
                double fInvLength = 1.0f / fLength;
                x *= fInvLength;
                y *= fInvLength;
                z *= fInvLength;
            }
            return fLength;
        }

		inline Vector3 normalized()
        {
        	normalise();
        	return Vector3(x, y, z);
        }

        inline void makeFloor( const Vector3& cmp )
        {
            if( cmp.x < x ) x = cmp.x;
            if( cmp.y < y ) y = cmp.y;
            if( cmp.z < z ) z = cmp.z;
        }

        inline void makeCeil( const Vector3& cmp )
        {
            if( cmp.x > x ) x = cmp.x;
            if( cmp.y > y ) y = cmp.y;
            if( cmp.z > z ) z = cmp.z;
        }

        inline Vector3 perpendicular() const
        {
            static const double fSquareZero = (double)(1e-06 * 1e-06);
            Vector3 perp = this->crossProduct( Vector3::UNIT_X );
            if( perp.squaredLength() < fSquareZero )
                perp = this->crossProduct( Vector3::UNIT_Y ); // This vector is the Y axis multiplied by a scalar, so we have to use another axis.

			perp.normalise();
            return perp;
        }

        inline bool isZeroLength() const {  return ((x * x) + (y * y) + (z * z) < (1e-06 * 1e-06)); }

        inline Vector3 normalisedCopy() const
        {
            Vector3 ret = *this;
            ret.normalise();
            return ret;
        }

        static const Vector3 ZERO;
        static const Vector3 UNIT_X;
        static const Vector3 UNIT_Y;
        static const Vector3 UNIT_Z;
        static const Vector3 UNIT_SCALE;
    };

	class Vector4
	{
	public:
		double x, y, z, w;

	public:
		inline Vector4() { }
		inline Vector4(const double fX, const double fY, const double fZ, const double fW) : x(fX), y(fY), z(fZ), w(fW) { }
		inline explicit Vector4(const double afCoordinate[4]) : x(afCoordinate[0]), y(afCoordinate[1]), z(afCoordinate[2]), w(afCoordinate[3]) { }
		inline explicit Vector4(const Vector3& rhs) : x(rhs.x), y(rhs.y), z(rhs.z), w(1.0f) { }

		inline double* ptr() { return &x; }
		inline const double* ptr() const { return &x; }

		inline Vector4& operator = (const Vector4& rkVector)
		{
			x = rkVector.x;
			y = rkVector.y;
			z = rkVector.z;
			w = rkVector.w;
			return *this;
		}

		inline bool operator == (const Vector4& rkVector) const { return (x == rkVector.x && y == rkVector.y && z == rkVector.z && w == rkVector.w); }
		inline bool operator != (const Vector4& rkVector) const { return (x != rkVector.x || y != rkVector.y || z != rkVector.z || w != rkVector.w); }

		inline Vector4& operator = (const Vector3& rhs)
		{
			x = rhs.x;
			y = rhs.y;
			z = rhs.z;
			w = 1.0f;
			return *this;
		}

		inline Vector4 operator + (const Vector4& rkVector) const
		{
			return Vector4(x + rkVector.x, y + rkVector.y, z + rkVector.z, w + rkVector.w);
		}

		inline Vector4 operator - (const Vector4& rkVector) const
		{
			return Vector4(x - rkVector.x, y - rkVector.y, z - rkVector.z, w - rkVector.w);
		}

		inline Vector4 operator * (const double fScalar) const
		{
			return Vector4(x * fScalar, y * fScalar, z * fScalar, w * fScalar);
		}

		inline Vector4 operator / (const double fScalar) const
		{
			assert(fScalar != 0.0);
			double fInv = 1.0f / fScalar;
			return Vector4(x * fInv, y * fInv, z * fInv, w * fInv);
		}

		inline const Vector4& operator + () const
		{
			return *this;
		}

		inline Vector4 operator - () const
		{
			return Vector4(-x, -y, -z, -w);
		}

		inline friend Vector4 operator * (const double fScalar, const Vector4& rkVector)
		{
			return Vector4(fScalar * rkVector.x, fScalar * rkVector.y, fScalar * rkVector.z, fScalar * rkVector.w);
		}

		inline Vector4& operator += (const Vector4& rkVector)
		{
			x += rkVector.x;
			y += rkVector.y;
			z += rkVector.z;
			w += rkVector.w;
			return *this;
		}

		inline Vector4& operator -= (const Vector4& rkVector)
		{
			x -= rkVector.x;
			y -= rkVector.y;
			z -= rkVector.z;
			w -= rkVector.w;
			return *this;
		}

		inline Vector4& operator *= (const double fScalar)
		{
			x *= fScalar;
			y *= fScalar;
			z *= fScalar;
			w *= fScalar;
			return *this;
		}

		inline Vector4& operator /= (const double fScalar)
		{
			assert(fScalar != 0.0);
			double fInv = 1.0f / fScalar;
			x *= fInv;
			y *= fInv;
			z *= fInv;
			w *= fInv;
			return *this;
		}

		inline double dotProduct(const Vector4& vec) const
		{
			return x * vec.x + y * vec.y + z * vec.z + w * vec.w;
		}
	};
}
#endif
