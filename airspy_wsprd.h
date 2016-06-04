/* 
 * FreeBSD License
 * Copyright (c) 2016, Guenael 
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */


#pragma once

#include <unistd.h>


#ifndef bool
    typedef int bool;
    #define true  1
    #define false 0
#endif


struct receiver_state {
    /* Variables used for stop conditions */
    bool exit_flag;
    bool record_flag;

    /* Buffer used for sampling */
    float   *idat;
    float   *qdat;
    unsigned int iqlen;  // FIXME uint32_t == iq_index...

    /* Variables used for decimation */
    uint32_t samples_to_xfer;
    uint32_t decim_index;
    uint32_t iq_index;
    int32_t  I_acc, Q_acc;
};


/* Option & config of the receiver */
struct receiver_options {
    int  freq;
    int  lnaGain;
    int  mixerGain;
    int  vgaGain;
    int  bias;
    int  ppm;
    int  rate;
    int  fs4;
    int  downsampling;
};
