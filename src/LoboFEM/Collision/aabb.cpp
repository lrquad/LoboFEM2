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

#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include "aabb.h"
#include <float.h>
#include<bits/stdc++.h> 

#define MAX(a,b)	((a) > (b) ? (a) : (b))
#define MIN(a,b)	((a) < (b) ? (a) : (b))

#define frand48()  (((float)rand()) / ((float) RAND_MAX))

void
vmin(vec3f &a, const vec3f &b)
{
	a[0] = MIN(a[0], b[0]);
	a[1] = MIN(a[1], b[1]);
	a[2] = MIN(a[2], b[2]);
}

void
vmax(vec3f &a, const vec3f &b)
{
	a[0] = MAX(a[0], b[0]);
	a[1] = MAX(a[1], b[1]);
	a[2] = MAX(a[2], b[2]);
}

void
aabb::empty()
{
	_max = vec3f(INT_MIN, INT_MIN, INT_MIN);
	_min = vec3f(INT_MAX, INT_MAX, INT_MAX);
}

aabb::aabb()
{
	empty();
	_color = vec3f(frand48(), frand48(), frand48());
}

aabb::aabb(const vec3f &v)
{
	_min = _max = v;
	_color = vec3f(frand48(), frand48(), frand48());
}

aabb &
aabb::operator += (const vec3d &p)
{
	_min[0] = MIN(_min[0], p.x);
	_min[1] = MIN(_min[1], p.y);
	_min[2] = MIN(_min[2], p.z);
	_max[0] = MAX(_max[0], p.x);
	_max[1] = MAX(_max[1], p.y);
	_max[2] = MAX(_max[2], p.z);
	return *this;
}

aabb &
aabb::operator += (const vec3f &p)
{
	vmin(_min, p);
	vmax(_max, p);
	return *this;
}

aabb &
aabb::operator += (const aabb &b)
{
	vmin(_min, b._min);
	vmax(_max, b._max);
	return *this;
}

bool
aabb::overlaps(const aabb& b) const
{
	if (_min[0] > b._max[0]) return false;
	if (_min[1] > b._max[1]) return false;
	if (_min[2] > b._max[2]) return false;

	if (_max[0] < b._min[0]) return false;
	if (_max[1] < b._min[1]) return false;
	if (_max[2] < b._min[2]) return false;

	return true;
}

bool
aabb::inside(const vec3f &p) const
{
	if (p[0] < _min[0] || p[0] > _max[0]) return false;
	if (p[1] < _min[1] || p[1] > _max[1]) return false;
	if (p[2] < _min[2] || p[2] > _max[2]) return false;

	return true;
}



void
aabb::visulization()
{
	
}
