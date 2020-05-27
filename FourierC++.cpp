// FourierC++.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>

#define _USE_MATH_DEFINES
#include <math.h>

#define sq(x) ((x)*(x))

const uint8_t delayTime = 1; // 1.1 - 0.9 = 1 1000KHz
const uint16_t MEETPUNTEN = 1024;
double real[MEETPUNTEN];
double img[MEETPUNTEN];

void loop();
void Swap(double *x, double *y);
uint8_t Exponent(uint16_t value);
void Compute(double *vReal, double *vImag, uint16_t samples, uint8_t power);
void DCRemoval(double *vData, uint16_t samples);
void ComplexToMagnitude(double *vReal, double *vImag, uint16_t samples);


int main()
{
    std::cout << "Hello World!\n";
	loop();
}

void loop() {

	for (int i = 0; i < MEETPUNTEN; i++) {
		real[i] = 100 * sin((double)((250.0 * M_PI)/(2 * M_PI) * i));
	}
	for (int i = 0; i < MEETPUNTEN; i++) {
		//std::cout << real[i] << std::endl;
	}
	std::cout << "FFT:" << std::endl;
	Compute(real, img, MEETPUNTEN, Exponent(MEETPUNTEN));
	ComplexToMagnitude(real, img, MEETPUNTEN);
	for (int i = 0; i < (MEETPUNTEN / 2); i++) {
		std::cout << real[i] << std::endl;//"Frequency: " << ((i * 1.0 * 1000) / MEETPUNTEN) << "	Amplitude: " << real[i] << std::endl;
	}
	printf("done");
}

void Swap(double *x, double *y) {
	double temp = *x;
	*x = *y;
	*y = temp;
}

uint8_t Exponent(uint16_t value) {
	uint8_t result = 0;
	while (((value >> result) & 1) != 1) result++;
	return(result);
}

void Compute(double *vReal, double *vImag, uint16_t samples, uint8_t power) { //, uint8_t dir) {
	uint16_t j = 0;
	for (uint16_t i = 0; i < (samples - 1); i++) {
		if (i < j) {
			Swap(&vReal[i], &vReal[j]);
			// if(dir==FFT_REVERSE)
			// 	Swap(&vImag[i], &vImag[j]);
		}
		uint16_t k = (samples >> 1);
		while (k <= j) {
			j -= k;
			k >>= 1;
		}
		j += k;
	}

	double c1 = -1.0;
	double c2 = 0.0;
	uint16_t l2 = 1;

	for (uint8_t l = 0; (l < power); l++) {
		uint16_t l1 = l2;
		l2 <<= 1;
		double u1 = 1.0;
		double u2 = 0.0;
		for (j = 0; j < l1; j++) {
			for (uint16_t i = j; i < samples; i += l2) {
				uint16_t i1 = i + l1;
				double t1 = u1 * vReal[i1] - u2 * vImag[i1];
				double t2 = u1 * vImag[i1] + u2 * vReal[i1];
				vReal[i1] = vReal[i] - t1;
				vImag[i1] = vImag[i] - t2;
				vReal[i] += t1;
				vImag[i] += t2;
			}
			double z = ((u1 * c1) - (u2 * c2));
			u2 = ((u1 * c2) + (u2 * c1));
			u1 = z;
		}

		c2 = sqrt((1.0 - c1) / 2.0);
		c1 = sqrt((1.0 + c1) / 2.0);

		// if (dir == FFT_FORWARD) {
		//c2 = -c2;
		// }
	}
	// Scaling for reverse transform
	// if (dir != FFT_FORWARD) {
	// 	for (uint16_t i = 0; i < samples; i++) {
	// 		 vReal[i] /= samples;
	// 		 vImag[i] /= samples;
	// 	}
	// }
}


void ComplexToMagnitude(double *vReal, double *vImag, uint16_t samples) {
	for (uint16_t i = 0; i < samples; i++) {
		vReal[i] = sqrt(sq(vReal[i]) + sq(vImag[i]));
	}
}

void DCRemoval(double *vData, uint16_t samples) {
	double mean = 0;
	for (uint16_t i = 1; i < ((samples >> 1) + 1); i++) {
		mean += vData[i];
	}

	mean /= samples;

	for (uint16_t i = 1; i < ((samples >> 1) + 1); i++) {
		vData[i] -= mean;
	}
}