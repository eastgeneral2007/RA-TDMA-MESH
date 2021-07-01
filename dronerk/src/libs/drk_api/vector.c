/**
 * vector.c
 * Nat Storer - 6/3/2011 
 * Perform simple vector_t operations for compass calculations 
 */

#include <math.h>

#include "drk/vector.h"

/* Cross Product */
void vector_cross(const vector_t *a,const vector_t *b, vector_t *out)
{
	out->x = a->y*b->z - a->z*b->y;
	out->y = a->z*b->x - a->x*b->z;
	out->z = a->x*b->y - a->y*b->x;
}

/* Dot Product */
float vector_dot(const vector_t *a,const vector_t *b)
{
	return a->x*b->x+a->y*b->y+a->z*b->z;
}

/* Normalize */
void vector_normalize(vector_t *a)
{
	float mag = sqrt(vector_dot(a,a));
	a->x /= mag;
	a->y /= mag;
	a->z /= mag;
}
