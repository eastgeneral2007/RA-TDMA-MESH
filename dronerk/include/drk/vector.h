/**
 * vector.h
 * Nat Storer - 6/3/2011 
 * Luis Pinto - since
 * Perform simple vector operations for compass calculations 
 */

#ifndef VECTOR_H
#define VECTOR_H

//

/* 3D vector */
typedef struct 
{
  double x, y, z;
} vector_t;

/* Cross Product */
void vector_cross(const vector_t *a, const vector_t *b, vector_t *out);

/* Dot Product */
float vector_dot(const vector_t *a,const vector_t *b);

/* Normalize */
void vector_normalize(vector_t *a);

#endif
