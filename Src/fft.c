/*
 * fft.c
 *
 *  Created on: 10 de jul de 2020
 *      Author: alves
 */

#include <math.h>
#include <stdint.h>
#include "cnumbers.h"
#include "fft.h"

static void cleaning(complex *z, uint32_t k);
static void rms(complex *z);

/*Calculates the discrete fourier transform and returns the address for a complex*/
complex *fft(float *x)
{
	static complex X[N];
	complex aux, Wnk, Z[N];
	float cosseno;

	cleaning(&aux, 1);
	cleaning(X, N);
	cleaning(Z, N);
	cleaning(&Wnk, 1);

	for(uint32_t k=0; k<K; k++)
	{
		Wnk.re=0;
		Wnk.im=0;
		Wnk.mod=-1;
		Wnk.pha=-2*PI/N*k;
		cverify(&Wnk);

		cosseno=2*cos(2*PI*k/N);

		Z[0].re=x[0] + cosseno*0 - 0;
		cverify(&Z[0]);

		Z[1].re=x[1] + cosseno*Z[0].re - 0;
		cverify(&Z[1]);

		for(uint32_t n=2; n<N; n++)
		{
			Z[n].re=x[n] + cosseno*Z[n-1].re - Z[n-2].re;
			cverify(&Z[n]);
		}

		aux=cdot(&Wnk, &Z[N-2]);
		X[k]=csum(&Z[N-1], &aux);

		//Cleaning
		cleaning(&aux, 1);
		cleaning(Z, N);
	}

	rms(X);

	return X;
}

/*Reset a complex variable*/
static void cleaning(complex *z, uint32_t k)
{
	for(uint32_t i=0; i<k; i++)
	{
		z[i].re=0;
		z[i].im=0;
		z[i].mod=0;
		z[i].pha=0;
	}
}

/*Calculates the rms value of a module of a dft*/
static void rms(complex *z)
{
	for(uint32_t i=0; i<N; i++)
	{
		if(i==0)
		{
			z[i].mod=(float)z[i].mod/N;
		}
		else
		{
			z[i].mod=(float)z[i].mod/(N/2);
			z[i].mod=(float)z[i].mod/sqrt(2);
		}
		z[i].re=0;
		z[i].im=0;
		cverify(&z[i]);
	}
}
