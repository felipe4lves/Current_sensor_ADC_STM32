/*
 * cnumbers.c
 *
 *  Created on: 8 de jul de 2020
 *      Author: alves
 */


#include "cnumbers.h"
#include <math.h>

static void csumverify(complex *z);
static void cdotverify(complex *z);

/* Adds complex numbers and adjust the mod and the phase*/
complex csum(complex *z1, complex *z2)
{
	complex result;

	result.re=z1->re+z2->re;
	result.im=z1->im+z2->im;

	csumverify(&result);

	return result;
}

/*Multiply complex numbers and adjust the real and imaginary parts*/
complex cdot(complex *z1, complex *z2)
{
	complex result;

	result.mod=z1->mod*z2->mod;
	result.pha=z1->pha+z2->pha;

	if(result.pha>PI)
		result.pha=-(2*PI-result.pha);
	else if(result.pha<-PI)
		result.pha=2*PI+result.pha;

	cdotverify(&result);

	return result;
}

/*adjust the mod and the phase*/
static void csumverify(complex *z)
{
	z->mod=sqrt(pow(z->re,2)+pow(z->im,2));
	if(z->re>0)
	{
		if(z->im>=0)
			z->pha=atan(z->im/z->re);
		else
			z->pha=-atan(-z->im/z->re);
	}
	else if(z->re<0)
	{
		if(z->im>=0)
			z->pha=PI-atan(-z->im/z->re);
		else
			z->pha=-PI+atan(z->im/z->re);
	}
	else
	{
		if(z->im>0)
			z->pha=PI/2;
		else if(z->im<0)
			z->pha=-PI/2;
		else
			z->pha=0;
	}


}

/*adjust the real and imaginary parts*/
static void cdotverify(complex *z)
{
	z->re=z->mod*cos(z->pha);
	z->im=z->mod*sin(z->pha);

	csumverify(z);

}

/*adjust the real and imaginary parts if the mod or phase are different from zero or adjust the mod and the phase
 * if the imaginary or real parts are different from zero*/
void cverify(complex *z)
{
	if(z->re==0 && z->im==0 && z->mod==0 && z->pha==0)
	{

	}
	else if(z->re!=0 || z->im!=0)
	{
		csumverify(z);
	}
	else if(z->mod!=0 || z->pha!=0)
	{
		cdotverify(z);
	}
}
