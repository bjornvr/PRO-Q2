/*
  LiquidCrystal Library - Hello World

 Demonstrates the use a 16x2 LCD display.  The LiquidCrystal
 library works with all LCD displays that are compatible with the
 Hitachi HD44780 driver. There are many of them out there, and you
 can usually tell them by the 16-pin interface.

 This sketch prints "Hello World!" to the LCD
 and shows the time.

  The circuit:
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 5
 * LCD D5 pin to digital pin 4
 * LCD D6 pin to digital pin 3
 * LCD D7 pin to digital pin 2
 * LCD R/W pin to ground
 * LCD VSS pin to ground
 * LCD VCC pin to 5V
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)

 Library originally added 18 Apr 2008
 by David A. Mellis
 library modified 5 Jul 2009
 by Limor Fried (http://www.ladyada.net)
 example added 9 Jul 2009
 by Tom Igoe
 modified 22 Nov 2010
 by Tom Igoe

 This example code is in the public domain.

 http://www.arduino.cc/en/Tutorial/LiquidCrystal
 */

 // include the library code:
#include <LiquidCrystal.h>
/*
	FFT libray
	Copyright (C) 2010 Didier Longueville
	Copyright (C) 2014 Enrique Condes
	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.
	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.
	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef arduinoFFT_h /* Prevent loading library twice */
#define arduinoFFT_h
#ifdef ARDUINO
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h" /* This is where the standard Arduino code lies */
#endif
#else
#include <stdlib.h>
#include <stdio.h>
#ifdef __AVR__
#include <avr/io.h>
#include <avr/pgmspace.h>
#endif
#include <math.h>
#include "defs.h"
#include "types.h"
#endif

#define FFT_LIB_REV 0x14
/* Custom constants */
#define FFT_FORWARD 0x01
#define FFT_REVERSE 0x00

/* Windowing type */
#define FFT_WIN_TYP_RECTANGLE 0x00 /* rectangle (Box car) */
#define FFT_WIN_TYP_HAMMING 0x01 /* hamming */
#define FFT_WIN_TYP_HANN 0x02 /* hann */
#define FFT_WIN_TYP_TRIANGLE 0x03 /* triangle (Bartlett) */
#define FFT_WIN_TYP_NUTTALL 0x04 /* nuttall */
#define FFT_WIN_TYP_BLACKMAN 0x05 /* blackman */
#define FFT_WIN_TYP_BLACKMAN_NUTTALL 0x06 /* blackman nuttall */
#define FFT_WIN_TYP_BLACKMAN_HARRIS 0x07 /* blackman harris*/
#define FFT_WIN_TYP_FLT_TOP 0x08 /* flat top */
#define FFT_WIN_TYP_WELCH 0x09 /* welch */
/*Mathematial constants*/
#define twoPi 6.28318531
#define fourPi 12.56637061
#define sixPi 18.84955593

#ifdef __AVR__
static const double _c1[]PROGMEM = { 0.0000000000, 0.7071067812, 0.9238795325, 0.9807852804,
															0.9951847267, 0.9987954562, 0.9996988187, 0.9999247018,
															0.9999811753, 0.9999952938, 0.9999988235, 0.9999997059,
															0.9999999265, 0.9999999816, 0.9999999954, 0.9999999989,
															0.9999999997 };
static const double _c2[]PROGMEM = { 1.0000000000, 0.7071067812, 0.3826834324, 0.1950903220,
															0.0980171403, 0.0490676743, 0.0245412285, 0.0122715383,
															0.0061358846, 0.0030679568, 0.0015339802, 0.0007669903,
															0.0003834952, 0.0001917476, 0.0000958738, 0.0000479369,
															0.0000239684 };
#endif
class arduinoFFT {
public:
	/* Constructor */
	arduinoFFT(void);
	arduinoFFT(double* vReal, double* vImag, uint16_t samples, double samplingFrequency);
	/* Destructor */
	~arduinoFFT(void);
	/* Functions */
	uint8_t Revision(void);
	uint8_t Exponent(uint16_t value);

	void ComplexToMagnitude(double* vReal, double* vImag, uint16_t samples);
	void Compute(double* vReal, double* vImag, uint16_t samples, uint8_t dir);
	void Compute(double* vReal, double* vImag, uint16_t samples, uint8_t power, uint8_t dir);
	void DCRemoval(double* vData, uint16_t samples);
	double MajorPeak(double* vD, uint16_t samples, double samplingFrequency);
	uint16_t Harmonic(double* vD, uint16_t samples, double samplingFrequency, double* f, double* v, uint16_t IndexOfMaxY);
	uint16_t MajorPeak(double* vD, uint16_t samples, double samplingFrequency, double* f, double* v);
	void Windowing(double* vData, uint16_t samples, uint8_t windowType, uint8_t dir);

	void ComplexToMagnitude();
	void Compute(uint8_t dir);
	void DCRemoval();
	double MajorPeak();
	void MajorPeak(double* f, double* v);
	void Windowing(uint8_t windowType, uint8_t dir);

private:
	/* Variables */
	uint16_t _samples;
	double _samplingFrequency;
	double* _vReal;
	double* _vImag;
	uint8_t _power;
	/* Functions */
	void Swap(double* x, double* y);
};

#endif

//Start of cpp file of library

arduinoFFT::arduinoFFT(void)
{ // Constructor
	#warning("This method is deprecated and may be removed on future revisions.")
}

arduinoFFT::arduinoFFT(double* vReal, double* vImag, uint16_t samples, double samplingFrequency)
{// Constructor
	this->_vReal = vReal;
	this->_vImag = vImag;
	this->_samples = samples;
	this->_samplingFrequency = samplingFrequency;
	this->_power = Exponent(samples);
}

arduinoFFT::~arduinoFFT(void)
{
	// Destructor
}

uint8_t arduinoFFT::Revision(void)
{
	return(FFT_LIB_REV);
}

void arduinoFFT::Compute(double* vReal, double* vImag, uint16_t samples, uint8_t dir)
{
	#warning("This method is deprecated and may be removed on future revisions.")
		Compute(vReal, vImag, samples, Exponent(samples), dir);
}

void arduinoFFT::Compute(uint8_t dir)
{// Computes in-place complex-to-complex FFT /
	// Reverse bits /
	uint16_t j = 0;
	for (uint16_t i = 0; i < (this->_samples - 1); i++) {
		if (i < j) {
			Swap(&this->_vReal[i], &this->_vReal[j]);
			if (dir == FFT_REVERSE)
				Swap(&this->_vImag[i], &this->_vImag[j]);
		}
		uint16_t k = (this->_samples >> 1);
		while (k <= j) {
			j -= k;
			k >>= 1;
		}
		j += k;
	}
	// Compute the FFT  /
#ifdef __AVR__
	uint8_t index = 0;
#endif
	double c1 = -1.0;
	double c2 = 0.0;
	uint16_t l2 = 1;
	for (uint8_t l = 0; (l < this->_power); l++) {
		uint16_t l1 = l2;
		l2 <<= 1;
		double u1 = 1.0;
		double u2 = 0.0;
		for (j = 0; j < l1; j++) {
			for (uint16_t i = j; i < this->_samples; i += l2) {
				uint16_t i1 = i + l1;
				double t1 = u1 * this->_vReal[i1] - u2 * this->_vImag[i1];
				double t2 = u1 * this->_vImag[i1] + u2 * this->_vReal[i1];
				this->_vReal[i1] = this->_vReal[i] - t1;
				this->_vImag[i1] = this->_vImag[i] - t2;
				this->_vReal[i] += t1;
				this->_vImag[i] += t2;
			}
			double z = ((u1 * c1) - (u2 * c2));
			u2 = ((u1 * c2) + (u2 * c1));
			u1 = z;
		}
#ifdef __AVR__
		c2 = pgm_read_float_near(&(_c2[index]));
		c1 = pgm_read_float_near(&(_c1[index]));
		index++;
#else
		c2 = sqrt((1.0 - c1) / 2.0);
		c1 = sqrt((1.0 + c1) / 2.0);
#endif
		if (dir == FFT_FORWARD) {
			c2 = -c2;
		}
	}
	// Scaling for reverse transform /
	if (dir != FFT_FORWARD) {
		for (uint16_t i = 0; i < this->_samples; i++) {
			this->_vReal[i] /= this->_samples;
			this->_vImag[i] /= this->_samples;
		}
	}
}

void arduinoFFT::Compute(double* vReal, double* vImag, uint16_t samples, uint8_t power, uint8_t dir)
{	// Computes in-place complex-to-complex FFT
	// Reverse bits
	#warning("This method is deprecated and may be removed on future revisions.")
		uint16_t j = 0;
	for (uint16_t i = 0; i < (samples - 1); i++) {
		if (i < j) {
			Swap(&vReal[i], &vReal[j]);
			if (dir == FFT_REVERSE)
				Swap(&vImag[i], &vImag[j]);
		}
		uint16_t k = (samples >> 1);
		while (k <= j) {
			j -= k;
			k >>= 1;
		}
		j += k;
	}
	// Compute the FFT
#ifdef __AVR__
	uint8_t index = 0;
#endif
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
#ifdef __AVR__
		c2 = pgm_read_float_near(&(_c2[index]));
		c1 = pgm_read_float_near(&(_c1[index]));
		index++;
#else
		c2 = sqrt((1.0 - c1) / 2.0);
		c1 = sqrt((1.0 + c1) / 2.0);
#endif
		if (dir == FFT_FORWARD) {
			c2 = -c2;
		}
	}
	// Scaling for reverse transform
	if (dir != FFT_FORWARD) {
		for (uint16_t i = 0; i < samples; i++) {
			vReal[i] /= samples;
			vImag[i] /= samples;
		}
	}
}

void arduinoFFT::ComplexToMagnitude()
{ // vM is half the size of vReal and vImag
	for (uint16_t i = 0; i < this->_samples; i++) {
		this->_vReal[i] = sqrt(sq(this->_vReal[i]) + sq(this->_vImag[i]));
	}
}

void arduinoFFT::ComplexToMagnitude(double* vReal, double* vImag, uint16_t samples)
{	// vM is half the size of vReal and vImag
	#warning("This method is deprecated and may be removed on future revisions.")
		for (uint16_t i = 0; i < samples; i++) {
			vReal[i] = sqrt(sq(vReal[i]) + sq(vImag[i]));
		}
}

void arduinoFFT::DCRemoval()
{
	// calculate the mean of vData
	double mean = 0;
	for (uint16_t i = 1; i < ((this->_samples >> 1) + 1); i++)
	{
		mean += this->_vReal[i];
	}
	mean /= this->_samples;
	// Subtract the mean from vData
	for (uint16_t i = 1; i < ((this->_samples >> 1) + 1); i++)
	{
		this->_vReal[i] -= mean;
	}
}

void arduinoFFT::DCRemoval(double* vData, uint16_t samples)
{
	// calculate the mean of vData
	#warning("This method is deprecated and may be removed on future revisions.")
		double mean = 0;
	for (uint16_t i = 1; i < ((samples >> 1) + 1); i++)
	{
		mean += vData[i];
	}
	mean /= samples;
	// Subtract the mean from vData
	for (uint16_t i = 1; i < ((samples >> 1) + 1); i++)
	{
		vData[i] -= mean;
	}
}

void arduinoFFT::Windowing(uint8_t windowType, uint8_t dir)
{// Weighing factors are computed once before multiple use of FFT
// The weighing function is symetric; half the weighs are recorded
	double samplesMinusOne = (double(this->_samples) - 1.0);
	for (uint16_t i = 0; i < (this->_samples >> 1); i++) {
		double indexMinusOne = double(i);
		double ratio = (indexMinusOne / samplesMinusOne);
		double weighingFactor = 1.0;
		// Compute and record weighting factor
		switch (windowType) {
		case FFT_WIN_TYP_RECTANGLE: // rectangle (box car)
			weighingFactor = 1.0;
			break;
		case FFT_WIN_TYP_HAMMING: // hamming
			weighingFactor = 0.54 - (0.46 * cos(twoPi * ratio));
			break;
		case FFT_WIN_TYP_HANN: // hann
			weighingFactor = 0.54 * (1.0 - cos(twoPi * ratio));
			break;
		case FFT_WIN_TYP_TRIANGLE: // triangle (Bartlett)
			weighingFactor = 1.0 - ((2.0 * abs(indexMinusOne - (samplesMinusOne / 2.0))) / samplesMinusOne);
			break;
		case FFT_WIN_TYP_NUTTALL: // nuttall
			weighingFactor = 0.355768 - (0.487396 * (cos(twoPi * ratio))) + (0.144232 * (cos(fourPi * ratio))) - (0.012604 * (cos(sixPi * ratio)));
			break;
		case FFT_WIN_TYP_BLACKMAN: // blackman
			weighingFactor = 0.42323 - (0.49755 * (cos(twoPi * ratio))) + (0.07922 * (cos(fourPi * ratio)));
			break;
		case FFT_WIN_TYP_BLACKMAN_NUTTALL: // blackman nuttall
			weighingFactor = 0.3635819 - (0.4891775 * (cos(twoPi * ratio))) + (0.1365995 * (cos(fourPi * ratio))) - (0.0106411 * (cos(sixPi * ratio)));
			break;
		case FFT_WIN_TYP_BLACKMAN_HARRIS: // blackman harris
			weighingFactor = 0.35875 - (0.48829 * (cos(twoPi * ratio))) + (0.14128 * (cos(fourPi * ratio))) - (0.01168 * (cos(sixPi * ratio)));
			break;
		case FFT_WIN_TYP_FLT_TOP: // flat top
			weighingFactor = 0.2810639 - (0.5208972 * cos(twoPi * ratio)) + (0.1980399 * cos(fourPi * ratio));
			break;
		case FFT_WIN_TYP_WELCH: // welch
			weighingFactor = 1.0 - sq((indexMinusOne - samplesMinusOne / 2.0) / (samplesMinusOne / 2.0));
			break;
		}
		if (dir == FFT_FORWARD) {
			this->_vReal[i] *= weighingFactor;
			this->_vReal[this->_samples - (i + 1)] *= weighingFactor;
		}
		else {
			this->_vReal[i] /= weighingFactor;
			this->_vReal[this->_samples - (i + 1)] /= weighingFactor;
		}
	}
}


void arduinoFFT::Windowing(double* vData, uint16_t samples, uint8_t windowType, uint8_t dir)
{// Weighing factors are computed once before multiple use of FFT
// The weighing function is symetric; half the weighs are recorded
	#warning("This method is deprecated and may be removed on future revisions.")
		double samplesMinusOne = (double(samples) - 1.0);
	for (uint16_t i = 0; i < (samples >> 1); i++) {
		double indexMinusOne = double(i);
		double ratio = (indexMinusOne / samplesMinusOne);
		double weighingFactor = 1.0;
		// Compute and record weighting factor
		switch (windowType) {
		case FFT_WIN_TYP_RECTANGLE: // rectangle (box car)
			weighingFactor = 1.0;
			break;
		case FFT_WIN_TYP_HAMMING: // hamming
			weighingFactor = 0.54 - (0.46 * cos(twoPi * ratio));
			break;
		case FFT_WIN_TYP_HANN: // hann
			weighingFactor = 0.54 * (1.0 - cos(twoPi * ratio));
			break;
		case FFT_WIN_TYP_TRIANGLE: // triangle (Bartlett)
			weighingFactor = 1.0 - ((2.0 * abs(indexMinusOne - (samplesMinusOne / 2.0))) / samplesMinusOne);
			break;
		case FFT_WIN_TYP_NUTTALL: // nuttall
			weighingFactor = 0.355768 - (0.487396 * (cos(twoPi * ratio))) + (0.144232 * (cos(fourPi * ratio))) - (0.012604 * (cos(sixPi * ratio)));
			break;
		case FFT_WIN_TYP_BLACKMAN: // blackman
			weighingFactor = 0.42323 - (0.49755 * (cos(twoPi * ratio))) + (0.07922 * (cos(fourPi * ratio)));
			break;
		case FFT_WIN_TYP_BLACKMAN_NUTTALL: // blackman nuttall
			weighingFactor = 0.3635819 - (0.4891775 * (cos(twoPi * ratio))) + (0.1365995 * (cos(fourPi * ratio))) - (0.0106411 * (cos(sixPi * ratio)));
			break;
		case FFT_WIN_TYP_BLACKMAN_HARRIS: // blackman harris
			weighingFactor = 0.35875 - (0.48829 * (cos(twoPi * ratio))) + (0.14128 * (cos(fourPi * ratio))) - (0.01168 * (cos(sixPi * ratio)));
			break;
		case FFT_WIN_TYP_FLT_TOP: // flat top
			weighingFactor = 0.2810639 - (0.5208972 * cos(twoPi * ratio)) + (0.1980399 * cos(fourPi * ratio));
			break;
		case FFT_WIN_TYP_WELCH: // welch
			weighingFactor = 1.0 - sq((indexMinusOne - samplesMinusOne / 2.0) / (samplesMinusOne / 2.0));
			break;
		}
		if (dir == FFT_FORWARD) {
			vData[i] *= weighingFactor;
			vData[samples - (i + 1)] *= weighingFactor;
		}
		else {
			vData[i] /= weighingFactor;
			vData[samples - (i + 1)] /= weighingFactor;
		}
	}
}

double arduinoFFT::MajorPeak()
{
	double maxY = 0;
	uint16_t IndexOfMaxY = 0;
	//If sampling_frequency = 2 * max_frequency in signal,
	//value would be stored at position samples/2
	for (uint16_t i = 1; i < ((this->_samples >> 1) + 1); i++) {
		if ((this->_vReal[i - 1] < this->_vReal[i]) && (this->_vReal[i] > this->_vReal[i + 1])) {
			if (this->_vReal[i] > maxY) {
				maxY = this->_vReal[i];
				IndexOfMaxY = i;
			}
		}
	}
	double delta = 0.5 * ((this->_vReal[IndexOfMaxY - 1] - this->_vReal[IndexOfMaxY + 1]) / (this->_vReal[IndexOfMaxY - 1] - (2.0 * this->_vReal[IndexOfMaxY]) + this->_vReal[IndexOfMaxY + 1]));
	double interpolatedX = ((IndexOfMaxY + delta) * this->_samplingFrequency) / (this->_samples - 1);
	if (IndexOfMaxY == (this->_samples >> 1)) //To improve calculation on edge values
		interpolatedX = ((IndexOfMaxY + delta) * this->_samplingFrequency) / (this->_samples);
	// returned value: interpolated frequency peak apex
	return(interpolatedX);
}

void arduinoFFT::MajorPeak(double* f, double* v)
{
	double maxY = 0;
	uint16_t IndexOfMaxY = 0;
	//If sampling_frequency = 2 * max_frequency in signal,
	//value would be stored at position samples/2
	for (uint16_t i = 1; i < ((this->_samples >> 1) + 1); i++) {
		if ((this->_vReal[i - 1] < this->_vReal[i]) && (this->_vReal[i] > this->_vReal[i + 1])) {
			if (this->_vReal[i] > maxY) {
				maxY = this->_vReal[i];
				IndexOfMaxY = i;
			}
		}
	}
	double delta = 0.5 * ((this->_vReal[IndexOfMaxY - 1] - this->_vReal[IndexOfMaxY + 1]) / (this->_vReal[IndexOfMaxY - 1] - (2.0 * this->_vReal[IndexOfMaxY]) + this->_vReal[IndexOfMaxY + 1]));
	double interpolatedX = ((IndexOfMaxY + delta) * this->_samplingFrequency) / (this->_samples - 1);
	if (IndexOfMaxY == (this->_samples >> 1)) //To improve calculation on edge values
		interpolatedX = ((IndexOfMaxY + delta) * this->_samplingFrequency) / (this->_samples);
	// returned value: interpolated frequency peak apex
	*f = interpolatedX;
	*v = abs(this->_vReal[IndexOfMaxY - 1] - (2.0 * this->_vReal[IndexOfMaxY]) + this->_vReal[IndexOfMaxY + 1]);
}

double arduinoFFT::MajorPeak(double* vD, uint16_t samples, double samplingFrequency)
{
	double maxY = 0;
	uint16_t IndexOfMaxY = 0;
	//If sampling_frequency = 2 * max_frequency in signal,
	//value would be stored at position samples/2
	for (uint16_t i = 1; i < ((samples >> 1) + 1); i++) {
		if ((vD[i - 1] < vD[i]) && (vD[i] > vD[i + 1])) {
			if (vD[i] > maxY) {
				maxY = vD[i];
				IndexOfMaxY = i;
			}
		}
	}
	double delta = 0.5 * ((vD[IndexOfMaxY - 1] - vD[IndexOfMaxY + 1]) / (vD[IndexOfMaxY - 1] - (2.0 * vD[IndexOfMaxY]) + vD[IndexOfMaxY + 1]));
	double interpolatedX = ((IndexOfMaxY + delta) * samplingFrequency) / (samples - 1);
	if (IndexOfMaxY == (samples >> 1)) //To improve calculation on edge values
		interpolatedX = ((IndexOfMaxY + delta) * samplingFrequency) / (samples);
	// returned value: interpolated frequency peak apex
	return(interpolatedX);
}

uint16_t arduinoFFT::Harmonic(double* vD, uint16_t samples, double samplingFrequency, double* f, double* v, uint16_t IndexOfMaxY)
{

	double maxY = 0;
	for (uint16_t i = IndexOfMaxY + 2; i < ((samples >> 1) + 1); i++) {
		if ((vD[i - 1] < vD[i]) && (vD[i] > vD[i + 1])) {
			if (vD[i] > maxY) {
				maxY = vD[i];
				IndexOfMaxY = i;
			}
		}
	}

	double delta = 0.5 * ((vD[IndexOfMaxY - 1] - vD[IndexOfMaxY + 1]) / (vD[IndexOfMaxY - 1] - (2.0 * vD[IndexOfMaxY]) + vD[IndexOfMaxY + 1]));
	double interpolatedX = ((IndexOfMaxY + delta) * samplingFrequency) / (samples - 1);
	//double popo =
	if (IndexOfMaxY == (samples >> 1)) //To improve calculation on edge values
		interpolatedX = ((IndexOfMaxY + delta) * samplingFrequency) / (samples);
	// returned value: interpolated frequency peak apex
	*f = interpolatedX;
	*v = abs(vD[IndexOfMaxY - 1] - (2.0 * vD[IndexOfMaxY]) + vD[IndexOfMaxY + 1]);
	return IndexOfMaxY;

}

uint16_t arduinoFFT::MajorPeak(double* vD, uint16_t samples, double samplingFrequency, double* f, double* v)
{
	#warning("This method is deprecated and may be removed on future revisions.")
		double maxY = 0;
	uint16_t IndexOfMaxY = 0;
	//If sampling_frequency = 2 * max_frequency in signal,
	//value would be stored at position samples/2
	for (uint16_t i = 1; i < ((samples >> 1) + 1); i++) {
		if ((vD[i - 1] < vD[i]) && (vD[i] > vD[i + 1])) {
			if (vD[i] > maxY) {
				maxY = vD[i];
				IndexOfMaxY = i;
			}
		}
	}
	double delta = 0.5 * ((vD[IndexOfMaxY - 1] - vD[IndexOfMaxY + 1]) / (vD[IndexOfMaxY - 1] - (2.0 * vD[IndexOfMaxY]) + vD[IndexOfMaxY + 1]));
	double interpolatedX = ((IndexOfMaxY + delta) * samplingFrequency) / (samples - 1);
	//double popo =
	if (IndexOfMaxY == (samples >> 1)) //To improve calculation on edge values
		interpolatedX = ((IndexOfMaxY + delta) * samplingFrequency) / (samples);
	// returned value: interpolated frequency peak apex
	*f = interpolatedX;
	*v = abs(vD[IndexOfMaxY - 1] - (2.0 * vD[IndexOfMaxY]) + vD[IndexOfMaxY + 1]);
	return IndexOfMaxY;
}

uint8_t arduinoFFT::Exponent(uint16_t value)
{
	#warning("This method may not be accessible on future revisions.")
		// Calculates the base 2 logarithm of a value
		uint8_t result = 0;
	while (((value >> result) & 1) != 1) result++;
	return(result);
}

// Private functions

void arduinoFFT::Swap(double* x, double* y)
{
	double temp = *x;
	*x = *y;
	*y = temp;
}

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
/*
These values can be changed in order to evaluate the functions
*/
const uint16_t samples = 128; //This value MUST ALWAYS be a power of 2
double signalFrequency = 500;
double samplingFrequency = 1000;
unsigned int sampling_period_us;
bool highspeed = false;
#define DIVH 38.0
#define DIVL 44.0
double divider = DIVL;

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03


void setup() {
	// set up the LCD's number of columns and rows:
	lcd.begin(16, 2);
	// Print a message to the LCD.
	lcd.print("Hello! PRO-Q2");


	// Calculate sampling rate
	sampling_period_us = round(1000000 * (1.0 / samplingFrequency));
	analogReference(EXTERNAL);

	// Add serial for debugging
	Serial.begin(115200);
	Serial.println("Ready");

}


void loop()
{
	//GET RAW DATA
	unsigned long microseconds;
	for (int i = 0; i < samples; i++) {
		microseconds = micros();
		vReal[i] = analogRead(0);
		vImag[i] = 0;

		while (micros() < (microseconds + sampling_period_us)) {
		}
	}
	/* Build raw data */
	FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */
	FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
	FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */

  //  PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);
	double da, va, db, vb, dc, vc; //Position 1-3 and Value 1-3

	//Calculate first peak
	uint16_t pIndex = FFT.MajorPeak(vReal, samples, samplingFrequency, &da, &va);
	int pa = (int)da;
	pa = (pa / 10) * 10;

	//Calculate second peak
	pIndex = FFT.Harmonic(vReal, samples, samplingFrequency, &db, &vb, pIndex);
	int pb = (int)db;
	pb = (pb / 10) * 10;

	//Calculate second peak
	pIndex = FFT.Harmonic(vReal, samples, samplingFrequency, &dc, &vc, pIndex);
	int pc = (int)dc;
	pc = (pc / 10) * 10;


	//Serial print first 3 peaks
	Serial.println("Computed peaks:");
	Serial.print(da);
	Serial.print(" Hz, ");
	Serial.println(va, 3);
	//divider=38.0;
	divider = va / 100.0;
	va = va / divider;

	Serial.print(db, 3);
	Serial.print(" Hz, ");
	Serial.println(vb, 3);
	vb = vb / divider;

	Serial.print(dc, 3);
	Serial.print(" Hz, ");
	Serial.println(vc, 3);
	vc = vc / divider;


	// set the cursor to column 0, line 1
	lcd.setCursor(0, 0);
	// print highest value
	lcd.print("F(");

	lcd.print(pa, DEC);
	if (va < 95) {
		lcd.print("@.");
		lcd.print(va, 0);
	}
	else {
		lcd.print("@1.0");
	}
	lcd.print("V)");
	if (pa < 100) {
		lcd.print(" ");
	}

	//print first harmonic
	if (pa > 365) {
		lcd.print("     ");
		lcd.setCursor(0, 1);
		lcd.print("A+B(1000+)   ");
	}
	else {
		lcd.print("A(");
		lcd.print(pb, DEC);
		if (pb < 100) {
			lcd.print("@    ");
			lcd.setCursor(0, 1);
			lcd.print(".");
		}
		else {
			lcd.setCursor(0, 1);
			lcd.print("@.");
		}
		lcd.print(vb, 0);

		//print second harmonic
		if (pa > 200) {
			lcd.print(") B( 1000+ )       ");
		}
		else if (pc < 975) {
			lcd.print(")B(");
			lcd.print(pc, DEC);
		}
		else {
			lcd.print(")B(980+");
		}
		lcd.print("@.");
		if (vc < 10)
		{
			lcd.print("00");
		}
		lcd.print(vc, 0);
		lcd.print(")     ");
	}
	delay(200); /* Repeat after delay */

	//Switch between low frequency and high frequency mode
	if (pa > 100) {
		highspeed = true;
		Serial.println("HIGH");
	}
	if (pa < 120) {
		highspeed = false;
		Serial.println("LOW");
	}

	//Setup low frequency and high frequency mode
	if (highspeed) {
		signalFrequency = 1000;
		samplingFrequency = 2000;
		//    divider		      = DIVH;
	}
	else {
		signalFrequency = 500;
		samplingFrequency = 1000;
		//    divider           = DIVL;
	}
	sampling_period_us = round(1000000 * (1.0 / samplingFrequency));
}

void PrintVector(double* vData, uint16_t bufferSize, uint8_t scaleType)
{
	for (uint16_t i = 0; i < bufferSize; i++)
	{
		double abscissa;
		/* Print abscissa value */
		switch (scaleType)
		{
		case SCL_INDEX:
			abscissa = (i * 1.0);
			break;
		case SCL_TIME:
			abscissa = ((i * 1.0) / samplingFrequency);
			break;
		case SCL_FREQUENCY:
			abscissa = ((i * 1.0 * samplingFrequency) / samples);
			break;
		}
		Serial.print(abscissa, 6);
		if (scaleType == SCL_FREQUENCY)
			Serial.print(" Hz");
		Serial.print(" ");
		Serial.println(vData[i], 4);
	}
	Serial.println();
}