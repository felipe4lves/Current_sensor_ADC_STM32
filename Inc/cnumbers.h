/*
 * cnumbers.h
 *
 *  Created on: 8 de jul de 2020
 *      Author: alves
 */

#ifndef CNUMBERS_H_
#define CNUMBERS_H_

#define PI 3.141592654
typedef struct
{
	float re;
	float im;
	float mod;
	float pha;
}complex;

complex csum(complex *z1, complex *z2);
complex cdot(complex *z1, complex *z2);
void cverify(complex *z);

#endif /* CNUMBERS_H_ */
