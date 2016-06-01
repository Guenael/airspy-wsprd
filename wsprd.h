/*
 This file is part of program wsprd, a detector/demodulator/decoder
 for the Weak Signal Propagation Reporter (WSPR) mode.

 File name: wsprd.c

 Copyright 2001-2015, Joe Taylor, K1JT

 Much of the present code is based on work by Steven Franke, K9AN,
 which in turn was based on earlier work by K1JT.

 Copyright 2014-2015, Steven Franke, K9AN

 License: GNU GPL v3

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


#pragma once


// Shared pointer between rx & decoding code
double *idat;
double *qdat;

// Option & config
struct wsprd_options {
    int  freq;
    char rcall[13];  // FIXME -- buffer overflow
    char rloc[7];
    int  lnaGain;
    int  mixerGain;
    int  vgaGain;
    int  bias;
    int  ppm;
    char date[7];
    char uttime[5];
};
struct wsprd_options options;


void sync_and_demodulate(double *id, double *qd, long np,
                         unsigned char *symbols, float *f1, float fstep,
                         int *shift1, int lagmin, int lagmax, int lagstep,
                         float *drift1, int symfac, float *sync, int mode);
void subtract_signal(double *id, double *qd, long np,
                     float f0, int shift0, float drift0, unsigned char* channel_symbols);
void subtract_signal2(double *id, double *qd, long np,
                      float f0, int shift0, float drift0, unsigned char* channel_symbols);
int wspr_decode(unsigned int npoints);


