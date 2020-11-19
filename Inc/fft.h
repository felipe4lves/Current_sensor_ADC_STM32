/*
 * fft.h
 *
 *  Created on: 10 de jul de 2020
 *      Author: alves
 */

#ifndef INC_FFT_H_
#define INC_FFT_H_

#define N 450 //Quantidade de amostras
#define K 32 //Quantidade de frequências a serem processadas
#include "cnumbers.h"
complex *fft(float *x);

#endif /* INC_FFT_H_ */
