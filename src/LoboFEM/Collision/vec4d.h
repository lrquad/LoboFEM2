/*************************************************************************\

  Copyright 2007 The University of North Carolina at Chapel Hill.
  All Rights Reserved.

  Permission to use, copy, modify and distribute this software and its
  documentation for educational, research and non-profit purposes, without
   fee, and without a written agreement is hereby granted, provided that the
  above copyright notice and the following three paragraphs appear in all
  copies.

  IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL BE
  LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
  CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE
  USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY
  OF NORTH CAROLINA HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGES.

  THE UNIVERSITY OF NORTH CAROLINA SPECIFICALLY DISCLAIM ANY
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE
  PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND THE UNIVERSITY OF
  NORTH CAROLINA HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT,
  UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

  The authors may be contacted via:

  US Mail:             GAMMA Research Group at UNC
                       Department of Computer Science
                       Sitterson Hall, CB #3175
                       University of N. Carolina
                       Chapel Hill, NC 27599-3175

  Phone:               (919)962-1749

  EMail:              {geom, tangm}@cs.unc.edu


\**************************************************************************/


#pragma once

#include <assert.h>
#include <math.h>

#define VEC_EPS float(10e-6)

class vec3f {
	//protected:
	

public:
	float  v[3];

	vec3f() {v[0]=0; v[1]=0; v[2]=0;}
	vec3f(float x, float y, float z)
	{ v[0] = x; v[1] = y; v[2] = z; }

	float & operator [] ( int i ) { return v[i]; }
	const float & operator [] ( int i ) const { return v[i]; }

	vec3f & operator += ( const vec3f & u )
	{ for(int i = 0; i < 3; i++) v[i] += u.v[i]; return *this;}
	vec3f operator + ( const vec3f &v) const
	{ vec3f rt(*this); return rt += v; }
	vec3f & operator *= ( float d )
	{ for(int i = 0; i < 3; i++) v[i] *= d; return *this;}
	vec3f operator * ( float d) const
	{ vec3f rt(*this); return rt *= d; }
	
	float normalize() { 
		float sum(0);
		
		for(int i = 0; i < 3; i++) 
			sum += v[i]*v[i];

		sum = float(sqrt(sum));
		if (sum > VEC_EPS)
			for(int i = 0; i < 3; i++) 
                    v[i] /= sum;
		return sum;
	}
};

class vec3d {
public:
	union {
		struct {
		double x, y, z;
		};
		struct {
		double v[3];
		};
	};

	vec3d ()
	{x=0; y=0; z=0;}

	vec3d(vec3f v)
	{
		x = v[0];
		y = v[1];
		z = v[2];
	}
	
	vec3d(const float * v)
	{
		x = v[0];
		y = v[1];
		z = v[2];
	}

	const double & operator [] ( int i ) const
	{
		switch (i) {
		case 0: return x;
		case 1: return y;
		case 2: return z;
		default: assert(0); return x;
		}
	}

	vec3d(double x, double y, double z)
	{
		this->x = x;
		this->y = y;
		this->z = z;
	}

	vec3d operator+ (const vec3d &v) const
	{
		return vec3d(x+v.x, y+v.y, z+v.z);
	}

	vec3d operator- (const vec3d &v) const
	{
		return vec3d(x-v.x, y-v.y, z-v.z);
	}

	vec3d operator *(float t) const
	{
		return vec3d(x*t, y*t, z*t);
	}

     // cross product
     const vec3d cross(const vec3d &vec) const
     {
          return vec3d(y*vec.z - z*vec.y, z*vec.x - x*vec.z, x*vec.y - y*vec.x);
     }
};