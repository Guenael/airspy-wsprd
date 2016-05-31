/* 
 * FreeBSD License
 * Copyright (c) 2016, Guenael 
 * All rights reserved. 
 *
 * This file is based on AirSpy project & HackRF project
 *   Copyright 2012 Jared Boone <jared@sharebrained.com>
 *   Copyright 2014-2015 Benjamin Vernoux <bvernoux@airspy.com>
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


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <errno.h>
#include <limits.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <sys/time.h>
#include <signal.h>

#include <libairspy/airspy.h>

#include "wsprd.h"


#define FLOAT32_EL_SIZE_BYTE	4        // 4bytes = 32bit float
#define SAMPLING_FREQUENCY      2500000
#define DOWNSAMPLE				6667     //6667
#define CAPTURE_LENGHT          116


#ifndef bool
	typedef int bool;
	#define true 1
	#define false 0
#endif


struct airspy_device* device = NULL;

// Variables used for stop conditions
volatile bool do_exit = false;
volatile bool stop_rx = false;


// Variables used for decimation (CIC algo)
uint32_t samples_to_xfer = SAMPLING_FREQUENCY * CAPTURE_LENGHT;  // 120 seconds at 2.5Msps
static uint32_t decim_index=0;
static double I=0.0, Q=0.0;
static uint32_t iq_index=0;


int rx_callback(airspy_transfer_t* transfer) {
    float *sigIn = (float*) transfer->samples;

    uint32_t samples_to_write = transfer->sample_count;
    if (samples_to_write >= samples_to_xfer) {
        samples_to_write = samples_to_xfer;
    }
    samples_to_xfer -= samples_to_write;

    /* Economic mixer @ fs/4 (upper band)
       At fs/4, sin and cosin calculation are no longueur necessary.
     
               0   | pi/2 |  pi  | 3pi/2
             ----------------------------
       sin =   0   |  1   |  0   |  -1  |
       cos =   1   |  0   | -1   |   0  |
      
       out_I = in_I * cos(x) - in_Q * sin(x)
       out_Q = in_Q * cos(x) + in_I * sin(x)
       (Weaver technique, keep the lower band) 

    float tmp;
    for (uint32_t i=0; i<samples_to_write; i+=8) {
        tmp = sigIn[i+3];
        sigIn[i+3] = - sigIn[i+2];
        sigIn[i+2] = tmp;

        sigIn[i+4] = - sigIn[i+4];
        sigIn[i+5] = - sigIn[i+5];

        tmp = sigIn[i+6];
        sigIn[i+6] = -sigIn[i+7];
        sigIn[i+7] = tmp;
    }
    */

    /* -- Simple square window decimator -- FIXME
    TODO    : Use/implement a clean fractional decimator (CIC+FIR)
    PROBLEM : Not perfect in timing & possible aliasing

    Error using this simple decimator = 0.0221 second
      Ideal = (256/375)*162 = 110.592 seconds
      This  = (256÷374.925014997)×162 = 110.6141184 seconds
    */
    uint32_t i=0;
    while (i < samples_to_write) {
        I += (double)sigIn[i*2];
        Q += (double)sigIn[i*2+1];
        i++;
        decim_index++;
        if (decim_index < DOWNSAMPLE) {
            continue;
        }
        idat[iq_index] = I;
        qdat[iq_index] = Q;
        decim_index = 0;
        I = 0.0;
        Q = 0.0;
        iq_index++;
    }
    return 0;
}


void sigint_callback_handler(int signum) {
    fprintf(stdout, "Caught signal %d\n", signum);
    do_exit = true;
    stop_rx = true;
}


void usage(void) {
    fprintf(stderr,
        "airspy_wsprd, a simple WSPR daemon for AirSpy receivers\n\n"
        "Use:\tairspy_wsprd -f frequency -c callsign -l locator [-options]\n"
        "\t-f dial frequency [Hz], check http://wsprnet.org/ for frequencies \n"
        "\t-c your callsign\n"
        "\t-g your locator grid\n"
        "\t[-l LNA gain [0-14] (default: 3)]\n"
        "\t[-m mixer gain [0-15] (default: 5)]\n"
        "\t[-v VGA gain [0-15] (default: 5)]\n"
        "\t[-b enable RF bias (default off)]\n"
        "\t[-p ppm_error (default: -60)]\n"
        "Example for my station:\n"
        "\tairspy_wsprd -f 144489000 -c VA2GKA -g FN35fm -l 10 -m 7 -v 7\n");
    exit(1);
}


int main(int argc, char** argv) {
    int opt;
	int result;
	int exit_code = EXIT_SUCCESS;

    // RX buffer allocation (120 sec max @ 375sps)
    idat=malloc(sizeof(double)*65536);
    qdat=malloc(sizeof(double)*65536);

    // Default value -- FIXME with getopt...
    options.lnaGain = 3;
    options.mixerGain = 5;
    options.vgaGain = 5;
    options.bias = 0;
    options.ppm = 0;

    while ((opt = getopt(argc, argv, "f:c:g:l:m:v:b:p")) != -1) {
        switch (opt) {
        case 'f': // Frequency
            options.freq = (int)atoi(optarg);
            break;
        case 'c': // Callsign
            sprintf(options.rcall, "%.12s", optarg);
            break;
        case 'g': // Locator / Grid
            sprintf(options.rloc, "%.6s", optarg);
            break;
        case 'l': // LNA gain
            options.lnaGain = (int)atoi(optarg);
            if (options.lnaGain < 0) options.lnaGain = 0;
            if (options.lnaGain > 14 ) options.lnaGain = 14;
            break;
        case 'm': // Mixer gain
            options.mixerGain = (int)atoi(optarg);
            if (options.mixerGain < 0) options.mixerGain = 0;
            if (options.mixerGain > 15) options.mixerGain = 15;
            break;
        case 'v': // VGA gain
            options.vgaGain = (int)atoi(optarg);
            if (options.vgaGain < 0) options.vgaGain = 0;
            if (options.vgaGain > 15) options.vgaGain = 15;
            break;
        case 'b': // Bias setting
            options.bias = (int)atoi(optarg);
            if (options.bias < 0) options.bias = 0;
            if (options.bias > 1) options.bias = 1;
            break;
        case 'p': // PPS correction
            options.ppm = (int)atoi(optarg);
            break;
        default:
            usage();
            break;
        }
    }

    if (options.freq == 0) {
        fprintf(stderr, "Please specify a dial frequency.\n");
        fprintf(stderr, " --help for usage...\n");
        exit(1);
    }

    if (options.rcall[0] == 0) {
        fprintf(stderr, "Please specify your callsign.\n");
        fprintf(stderr, " --help for usage...\n");
        exit(1);
    }

    if (options.rloc[0] == 0) {
        fprintf(stderr, "Please specify your locator.\n");
        fprintf(stderr, " --help for usage...\n");
        exit(1);
    }

    signal(SIGINT, &sigint_callback_handler);
    signal(SIGILL, &sigint_callback_handler);
    signal(SIGFPE, &sigint_callback_handler);
    signal(SIGSEGV, &sigint_callback_handler);
    signal(SIGTERM, &sigint_callback_handler);
    signal(SIGABRT, &sigint_callback_handler);

	result = airspy_init();
	if( result != AIRSPY_SUCCESS ) {
		printf("airspy_init() failed: %s (%d)\n", airspy_error_name(result), result);
		return EXIT_FAILURE;
	}

	result = airspy_open(&device);
	if( result != AIRSPY_SUCCESS ) {
		printf("airspy_open() failed: %s (%d)\n", airspy_error_name(result), result);
		airspy_exit();
		return EXIT_FAILURE;
	}

	result = airspy_set_sample_type(device, AIRSPY_SAMPLE_FLOAT32_IQ);
	if (result != AIRSPY_SUCCESS) {
		printf("airspy_set_sample_type() failed: %s (%d)\n", airspy_error_name(result), result);
		airspy_close(device);
		airspy_exit();
		return EXIT_FAILURE;
	}

	result = airspy_set_samplerate(device, SAMPLING_FREQUENCY);
	if (result != AIRSPY_SUCCESS) {
		printf("airspy_set_samplerate() failed: %s (%d)\n", airspy_error_name(result), result);
		airspy_close(device);
		airspy_exit();
		return EXIT_FAILURE;
	}

	result = airspy_set_rf_bias(device, options.bias);
	if( result != AIRSPY_SUCCESS ) {
		printf("airspy_set_rf_bias() failed: %s (%d)\n", airspy_error_name(result), result);
		airspy_close(device);
		airspy_exit();
		return EXIT_FAILURE;
	}

	result = airspy_set_vga_gain(device, options.vgaGain);
	if( result != AIRSPY_SUCCESS ) {
		printf("airspy_set_vga_gain() failed: %s (%d)\n", airspy_error_name(result), result);
	}

	result = airspy_set_mixer_gain(device, options.mixerGain);
	if( result != AIRSPY_SUCCESS ) {
		printf("airspy_set_mixer_gain() failed: %s (%d)\n", airspy_error_name(result), result);
	}

	result = airspy_set_lna_gain(device, options.lnaGain);
	if( result != AIRSPY_SUCCESS ) {
		printf("airspy_set_lna_gain() failed: %s (%d)\n", airspy_error_name(result), result);
	}
	
	result = airspy_set_freq(device, options.freq + 1500);  // Dial + 1500Hz
    //result = airspy_set_freq(device, 144490500 - 625000);
	if( result != AIRSPY_SUCCESS ) {
		printf("airspy_set_freq() failed: %s (%d)\n", airspy_error_name(result), result);
		airspy_close(device);
		airspy_exit();
		return EXIT_FAILURE;
	}

    // Print used parameter
    time_t rawtime;
    time ( &rawtime );
    struct tm *gtm = gmtime(&rawtime);
    printf("Starting airspy-wsprd (%04d-%02d-%02d, %02d:%02dz)\n", 
    gtm->tm_year + 1900, gtm->tm_mon + 1, gtm->tm_mday, gtm->tm_hour, gtm->tm_min);
    printf("  Frequency  : %d Hz\n", options.freq);
    printf("  Callsign   : %s\n", options.rcall);
    printf("  Locator    : %s\n", options.rloc);
    printf("  LNA gain   : %d dB\n", options.lnaGain);
    printf("  Mixer gain : %d dB\n", options.mixerGain);
    printf("  VGA gain   : %d dB\n", options.vgaGain);
    printf("  Bias       : %d\n", options.bias);

    while (!do_exit) {
        // Time Sync on 2 mins
        struct timeval lTime;
        gettimeofday(&lTime, NULL);

        uint32_t sec   = lTime.tv_sec % 120;
        uint32_t usec  = sec * 1000000 + lTime.tv_usec;
        uint32_t uwait = 120000000 - usec;
        printf("Wait for time sync (start in %d sec)\n", uwait/1000000);
        usleep(uwait);
        printf("SYNC! RX started\n");

        // Store the date at the begin of the frame
        time ( &rawtime );
        gtm = gmtime(&rawtime);
        sprintf(options.date,"%02d%02d%02d", gtm->tm_year - 100, gtm->tm_mon + 1, gtm->tm_mday);
        sprintf(options.uttime,"%02d%02d", gtm->tm_hour, gtm->tm_min+1); //FIXME not +1 legit...

        // Start to sampling
        result = airspy_start_rx(device, rx_callback, NULL);
        if( result != AIRSPY_SUCCESS ) {
            printf("airspy_start_rx() failed: %s (%d)\n", airspy_error_name(result), result);
            airspy_close(device);
            airspy_exit();
            return EXIT_FAILURE;
        }

        while( (airspy_is_streaming(device) == AIRSPY_TRUE) &&
            (stop_rx == false) )
        {
            if (samples_to_xfer == 0)
                stop_rx = true;
            else
                sleep(1);
        }

        result = airspy_stop_rx(device);
        if( result != AIRSPY_SUCCESS ) {
            printf("airspy_stop_rx() failed: %s (%d)\n", airspy_error_name(result), result);
        }

        printf("RX done! [Buffer size: %d]\n\n", iq_index);

        /* Decode the samples */
        if(!do_exit)
            wspr_decode(iq_index);

        // Rearm for a new RX
        decim_index=0;
        I=0.0;
        Q=0.0;
        iq_index=0;
        samples_to_xfer = SAMPLING_FREQUENCY * CAPTURE_LENGHT;
        stop_rx = false;
    }

	if(device != NULL)
	{
		result = airspy_close(device);
		if( result != AIRSPY_SUCCESS ) 
		{
			printf("airspy_close() failed: %s (%d)\n", airspy_error_name(result), result);
		}
		
		airspy_exit();
	}

    printf("Bye!\n");
	return exit_code;
}
