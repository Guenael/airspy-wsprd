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
#include <signal.h>
#include <math.h>
#include <string.h>
#include <sys/time.h>
#include <pthread.h>
#include <curl/curl.h>

#include <libairspy/airspy.h>

#include "airspy_wsprd.h"
#include "wsprd.h"


/* TODO
 - clean/fix samplerate selection
 - clean/fix serial number section
 - implement linear gain etc.
 - multispot report in one post
 - type fix (uint32_t etc..)
*/


#define CAPTURE_LENGHT    118
#define MAX_SAMPLES_SIZE  45000    // (120 sec max @ 375sps)


/* Global declaration for these structs */
struct receiver_state   rx_state;         
struct receiver_options rx_options;
struct decoder_options  dec_options;
struct decoder_results  dec_results[50];
struct airspy_device*   device = NULL;
airspy_read_partid_serialno_t readSerial;


/* Thread stuff for separate decoding */
struct decoder_state {
    pthread_t        thread;

    pthread_rwlock_t rw;
    pthread_cond_t   ready_cond;
    pthread_mutex_t  ready_mutex;
};
struct decoder_state dec;



/* Callback for each buffer received */
int rx_callback(airspy_transfer_t* transfer) {
    int16_t *sigIn = (int16_t*) transfer->samples;

    /* Do not process the samples if reception not started or over */
    if ( (rx_state.record_flag == false) || (rx_state.exit_flag == true) )
        return 0;

    /* Economic mixer @ fs/4 (upper band)
       At fs/4, sin and cosin calculation are no longueur necessary.

               0   | pi/2 |  pi  | 3pi/2
             ----------------------------
       sin =   0   |  1   |  0   |  -1  |
       cos =   1   |  0   | -1   |   0  |

       out_I = in_I * cos(x) - in_Q * sin(x)
       out_Q = in_Q * cos(x) + in_I * sin(x)
       (Weaver technique, keep the lower band)
    */
    int16_t tmp;
    for (uint32_t i=0; i<(transfer->sample_count); i+=8) {
        tmp = sigIn[i+3];
        sigIn[i+3] = -sigIn[i+2];
        sigIn[i+2] = tmp;

        sigIn[i+4] = -sigIn[i+4];
        sigIn[i+5] = -sigIn[i+5];

        tmp = sigIn[i+6];
        sigIn[i+6] = -sigIn[i+7];
        sigIn[i+7] = tmp;
    }

    /* Simple square window decimator 
       (could be not perfect in time for some sampling rate. 
       Ex: AirSpy vs AirSpy Mini, but works fine in practice)
    */
    uint32_t samples_to_write = transfer->sample_count / 2;  // IQ take 2 samples
    if (samples_to_write >= rx_state.samples_to_xfer) {
        samples_to_write =  rx_state.samples_to_xfer;
    }
    rx_state.samples_to_xfer -= samples_to_write;

    uint32_t i=0;
    while (i < samples_to_write) {
        rx_state.I_acc += (int32_t)sigIn[i*2];
        rx_state.Q_acc += (int32_t)sigIn[i*2+1];
        i++;

        rx_state.decim_index++;
        if (rx_state.decim_index < rx_options.downsampling) {
            continue;
        }

        /* Lock the buffer during writing */     // Overkill ?!
        pthread_rwlock_wrlock(&dec.rw);  
        rx_state.idat[rx_state.iq_index] = (float)rx_state.I_acc;
        rx_state.qdat[rx_state.iq_index] = (float)rx_state.Q_acc;
        pthread_rwlock_unlock(&dec.rw);

        rx_state.I_acc = 0;
        rx_state.Q_acc = 0;
        rx_state.decim_index = 0;
        rx_state.iq_index++;
    }

    if (rx_state.samples_to_xfer == 0) {
        rx_state.record_flag = false;
        printf("RX done! [Buffer size: %d]\n", rx_state.iq_index);


        /* Send a signal to the other thread to start the decoding */
        pthread_mutex_lock(&dec.ready_mutex); 
        pthread_cond_signal(&dec.ready_cond); 
        pthread_mutex_unlock(&dec.ready_mutex);
    }
    return 0;
}


void postSpots(int n_results) {
    CURL *curl;
    CURLcode res;
    char url[256];

    for (uint32_t i=0; i<n_results; i++) {
        sprintf(url,"http://wsprnet.org/post?function=wspr&rcall=%s&rgrid=%s&rqrg=%.6f&date=%s&time=%s&sig=%.0f&dt=%.1f&tqrg=%.6f&tcall=%s&tgrid=%s&dbm=%s&version=0.1_wsprd&mode=2",
                dec_options.rcall, dec_options.rloc, dec_results[i].freq, dec_options.date, dec_options.uttime,
                dec_results[i].snr, dec_results[i].dt, dec_results[i].freq,
                dec_results[i].call, dec_results[i].loc, dec_results[i].pwr);

        printf("Spot : %3.2f %4.2f %10.6f %2d  %-s\n",
               dec_results[i].snr, dec_results[i].dt, dec_results[i].freq,
               (int)dec_results[i].drift, dec_results[i].message);

        curl = curl_easy_init();
        if(curl) {
            curl_easy_setopt(curl, CURLOPT_URL, url);
            curl_easy_setopt(curl, CURLOPT_NOBODY, 1);
            res = curl_easy_perform(curl);

            if(res != CURLE_OK)
                fprintf(stderr, "curl_easy_perform() failed: %s\n",curl_easy_strerror(res));

            curl_easy_cleanup(curl);
        }
    }
}


static void *wsprDecoder(void *arg) {
    //struct decoder_state *d = arg; // FIXME, besoin du arg avec struc globals ??

    static float iSamples[MAX_SAMPLES_SIZE]={0};
    static float qSamples[MAX_SAMPLES_SIZE]={0};
    static uint32_t samples_len;
    int n_results=0;

    while (!rx_state.exit_flag) {
        pthread_mutex_lock(&dec.ready_mutex); 
        pthread_cond_wait(&dec.ready_cond, &dec.ready_mutex); 
        pthread_mutex_unlock(&dec.ready_mutex);

        if(rx_state.exit_flag)  // Abord case, final sig
            break;

        // Lock the buffer access and make copy
        pthread_rwlock_wrlock(&dec.rw);
        memcpy(iSamples, rx_state.idat, rx_state.iq_index * sizeof(float));
        memcpy(qSamples, rx_state.qdat, rx_state.iq_index * sizeof(float));
        samples_len = rx_state.iq_index;  // Overkill ?
        pthread_rwlock_unlock(&dec.rw);

        // Search & decode the signal
        wspr_decode(iSamples, qSamples, samples_len, dec_options, dec_results, &n_results);
        postSpots(n_results);

    }
    pthread_exit(NULL);
}


double atofs(char *s) {
    /* standard suffixes */
    char last;
    uint32_t len;
    double suff = 1.0;
    len = strlen(s);
    last = s[len-1];
    s[len-1] = '\0';
    switch (last) {
        case 'g':
        case 'G':
            suff *= 1e3;
        case 'm':
        case 'M':
            suff *= 1e3;
        case 'k':
        case 'K':
            suff *= 1e3;
            suff *= atof(s);
            s[len-1] = last;
            return suff;
    }
    s[len-1] = last;
    return atof(s);
}


int parse_u64(char* s, uint64_t* const value) {
    uint_fast8_t base = 10;
    char* s_end;
    uint64_t u64_value;

    if( strlen(s) > 2 ) {
        if( s[0] == '0' ) {
            if( (s[1] == 'x') || (s[1] == 'X') ) {
                base = 16;
                s += 2;
            } else if( (s[1] == 'b') || (s[1] == 'B') ) {
                base = 2;
                s += 2;
            }
        }
    }

    s_end = s;
    u64_value = strtoull(s, &s_end, base);
    if( (s != s_end) && (*s_end == 0) ) {
        *value = u64_value;
        return AIRSPY_SUCCESS;
    } else {
        return AIRSPY_ERROR_INVALID_PARAM;
    }
}


/* Reset flow control variable & decimation variables */
void initSampleStorage() {
    rx_state.record_flag = true;
    rx_state.samples_to_xfer = rx_options.rate * CAPTURE_LENGHT;
    rx_state.decim_index=0;
    rx_state.iq_index=0;
    rx_state.I_acc=0;
    rx_state.Q_acc=0;
}


/* Default options for the decoder */
void initDecoder_options() {
    dec_options.usehashtable = 1;
    dec_options.npasses = 2;
    dec_options.subtraction = 1;
    dec_options.fmin=-110.0;
    dec_options.fmax=110.0;
}


/* Default options for the receiver */
void initrx_options() {
    rx_options.lnaGain = 3;    // DEFAULT_LNA_GAIN
    rx_options.mixerGain = 5;  // DEFAULT_MIXER_GAIN
    rx_options.vgaGain = 5;    // DEFAULT_VGA_IF_GAIN
    rx_options.bias = 0;
    rx_options.shift = 0;
    rx_options.rate = 2500000;
    rx_options.serialnumber = 0;
}


void sigint_callback_handler(int signum) {
    fprintf(stdout, "Caught signal %d\n", signum);
    rx_state.exit_flag = true;
    rx_state.record_flag = false;
}


void usage(void) {
    fprintf(stderr,
            "airspy_wsprd, a simple WSPR daemon for AirSpy receivers\n\n"
            "Use:\tairspy_wsprd -f frequency -c callsign -l locator [options]\n"
            "\t-f dial frequency [(,k,M)Hz], check http://wsprnet.org/ for freq.\n"
            "\t-c your callsign (12 chars max)\n"
            "\t-g your locator grid (6 chars max)\n"
            "Receiver extra options:\n"
            "\t[-l LNA gain [0-14] (default: 3)]\n"
            "\t[-m MIXER gain [0-15] (default: 5)]\n"
            "\t[-v VGA gain [0-15] (default: 5)]\n"
            "\t[-b set Bias Tee[0-1], (default: 0 disabled)]\n"
            "\t[-r sampling rate[2.5M, 3M, 6M, 10M], (default: 2.5M)]\n"
            "\t[-p frequency correction (default: 0)]\n"
            "\t[-s S/N]: Open device with specified 64bits serial number\n"
            "Decoder extra options:\n"
            "\t[-H do not use (or update) the hash table\n"
            "\t[-Q quick mode - doesn't dig deep for weak signals\n"
            "\t[-S single pass mode, no subtraction (same as original wsprd)\n"
            "\t[-W wideband mode - decode signals within +/- 150 Hz of center\n"
            "Example:\n"
            "\tairspy_wsprd -f 144.489M -r 2.5M -c A1XYZ -g AB12cd -l 10 -m 7 -v 7\n");
    exit(1);
}


int main(int argc, char** argv) {
    uint32_t opt;
    uint32_t result;
    uint32_t exit_code = EXIT_SUCCESS;

    initrx_options();
    initDecoder_options();

    // RX buffer allocation (120 sec max @ 375sps)
    rx_state.idat=malloc(sizeof(float)*MAX_SAMPLES_SIZE);
    rx_state.qdat=malloc(sizeof(float)*MAX_SAMPLES_SIZE);
    rx_state.exit_flag   = false;
    rx_state.record_flag = false;

    if (argc <= 1)
        usage();

    while ((opt = getopt(argc, argv, "f:c:g:r:l:m:v:b:s:p:H:Q:S:W")) != -1) {
        switch (opt) {
        case 'f': // Frequency
            rx_options.dialfreq = (uint32_t)atofs(optarg);
            break;
        case 'c': // Callsign
            sprintf(dec_options.rcall, "%.12s", optarg);
            break;
        case 'g': // Locator / Grid
            sprintf(dec_options.rloc, "%.6s", optarg);
            break;
        case 'r': // sampling rate
            rx_options.rate = (uint32_t)atofs(optarg);
            break;
        case 'l': // LNA gain
            rx_options.lnaGain = (uint32_t)atoi(optarg);
            if (rx_options.lnaGain < 0) rx_options.lnaGain = 0;
            if (rx_options.lnaGain > 14 ) rx_options.lnaGain = 14;
            break;
        case 'm': // Mixer gain
            rx_options.mixerGain = (uint32_t)atoi(optarg);
            if (rx_options.mixerGain < 0) rx_options.mixerGain = 0;
            if (rx_options.mixerGain > 15) rx_options.mixerGain = 15;
            break;
        case 'v': // VGA gain
            rx_options.vgaGain = (uint32_t)atoi(optarg);
            if (rx_options.vgaGain < 0) rx_options.vgaGain = 0;
            if (rx_options.vgaGain > 15) rx_options.vgaGain = 15;
            break;
        case 'b': // Bias setting
            rx_options.bias = (uint32_t)atoi(optarg);
            if (rx_options.bias < 0) rx_options.bias = 0;
            if (rx_options.bias > 1) rx_options.bias = 1;
            break;
        case 's':
            parse_u64(optarg, &rx_options.serialnumber);
            break;
        case 'p': // PPS correction
            rx_options.shift = (int32_t)atoi(optarg);
            break;
        case 'H':
            dec_options.usehashtable = 0;
        case 'Q':
            dec_options.quickmode = 1;
        case 'S':
            dec_options.subtraction = 0; //single pass mode (same as original wsprd)
            dec_options.npasses = 1;              
        case 'W':
            dec_options.fmin = -150.0;
            dec_options.fmax = 150.0;
        default:
            usage();
            break;
        }
    }

    if (rx_options.dialfreq == 0) {
        fprintf(stderr, "Please specify a dial frequency.\n");
        fprintf(stderr, " --help for usage...\n");
        exit(1);
    }

    if (dec_options.rcall[0] == 0) {
        fprintf(stderr, "Please specify your callsign.\n");
        fprintf(stderr, " --help for usage...\n");
        exit(1);
    }

    if (dec_options.rloc[0] == 0) {
        fprintf(stderr, "Please specify your locator.\n");
        fprintf(stderr, " --help for usage...\n");
        exit(1);
    }

    // FIXME // TODO : sampling rate check

    /* Calcule decimation rate & frequency offset for fs/4 shift */
    rx_options.fs4 = rx_options.rate / 4;
    rx_options.downsampling = (uint32_t)ceil((double)rx_options.rate / 375.0);
    rx_options.reelfreq = rx_options.dialfreq + rx_options.shift;

    /* If something goes wrong... */
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

    if( rx_options.serialnumber ) {
        result = airspy_open_sn(&device, rx_options.serialnumber);
        if( result != AIRSPY_SUCCESS ) {
            printf("airspy_open_sn() failed: %s (%d)\n", airspy_error_name(result), result);
            airspy_exit();
            return EXIT_FAILURE;
        }
    } else {
        result = airspy_open(&device);
        if( result != AIRSPY_SUCCESS ) {
            printf("airspy_open() failed: %s (%d)\n", airspy_error_name(result), result);
            airspy_exit();
            return EXIT_FAILURE;
        }
    }

    result = airspy_set_sample_type(device, AIRSPY_SAMPLE_INT16_REAL);
    if (result != AIRSPY_SUCCESS) {
        printf("airspy_set_sample_type() failed: %s (%d)\n", airspy_error_name(result), result);
        airspy_close(device);
        airspy_exit();
        return EXIT_FAILURE;
    }

    result = airspy_set_samplerate(device, rx_options.rate);
    if (result != AIRSPY_SUCCESS) {
        printf("airspy_set_samplerate() failed: %s (%d)\n", airspy_error_name(result), result);
        airspy_close(device);
        airspy_exit();
        return EXIT_FAILURE;
    }

    result = airspy_set_rf_bias(device, rx_options.bias);
    if( result != AIRSPY_SUCCESS ) {
        printf("airspy_set_rf_bias() failed: %s (%d)\n", airspy_error_name(result), result);
        airspy_close(device);
        airspy_exit();
        return EXIT_FAILURE;
    }

    result = airspy_set_vga_gain(device, rx_options.vgaGain);
    if( result != AIRSPY_SUCCESS ) {
        printf("airspy_set_vga_gain() failed: %s (%d)\n", airspy_error_name(result), result);
    }

    result = airspy_set_mixer_gain(device, rx_options.mixerGain);
    if( result != AIRSPY_SUCCESS ) {
        printf("airspy_set_mixer_gain() failed: %s (%d)\n", airspy_error_name(result), result);
    }

    result = airspy_set_lna_gain(device, rx_options.lnaGain);
    if( result != AIRSPY_SUCCESS ) {
        printf("airspy_set_lna_gain() failed: %s (%d)\n", airspy_error_name(result), result);
    }

    result = airspy_set_freq(device, rx_options.reelfreq + rx_options.fs4 + 1500);  // Dial + offset + 1500Hz
    if( result != AIRSPY_SUCCESS ) {
        printf("airspy_set_freq() failed: %s (%d)\n", airspy_error_name(result), result);
        airspy_close(device);
        airspy_exit();
        return EXIT_FAILURE;
    }

    result = airspy_board_partid_serialno_read(device, &readSerial);
    if (result != AIRSPY_SUCCESS) {
            fprintf(stderr, "airspy_board_partid_serialno_read() failed: %s (%d)\n",
                airspy_error_name(result), result);
            airspy_close(device);
            airspy_exit();
            return EXIT_FAILURE;
    }

    /* Sampling run non-stop, for stability and sample are dropped or stored */
    result = airspy_start_rx(device, rx_callback, NULL);
    if( result != AIRSPY_SUCCESS ) {
        printf("airspy_start_rx() failed: %s (%d)\n", airspy_error_name(result), result);
        airspy_close(device);
        airspy_exit();
        return EXIT_FAILURE;
    }

    // Print used parameter
    time_t rawtime;
    time ( &rawtime );
    struct tm *gtm = gmtime(&rawtime);
    printf("Starting airspy-wsprd (%04d-%02d-%02d, %02d:%02dz) -- Version 0.1\n",
           gtm->tm_year + 1900, gtm->tm_mon + 1, gtm->tm_mday, gtm->tm_hour, gtm->tm_min);
    printf("  Dial freq. : %d Hz\n", rx_options.dialfreq);
    printf("  Reel freq. : %d Hz\n", rx_options.reelfreq);
    printf("  Rate       : %d Hz\n", rx_options.rate);
    printf("  Callsign   : %s\n", dec_options.rcall);
    printf("  Locator    : %s\n", dec_options.rloc);
    printf("  LNA gain   : %d dB\n", rx_options.lnaGain);
    printf("  Mixer gain : %d dB\n", rx_options.mixerGain);
    printf("  VGA gain   : %d dB\n", rx_options.vgaGain);
    printf("  Bias       : %d\n", rx_options.bias);
    printf("  S/N        : 0x%08X%08X\n", readSerial.serial_no[2], readSerial.serial_no[3]);

    // Create the thread and stuff for separate decoding
    pthread_rwlock_init(&dec.rw, NULL);
    pthread_cond_init(&dec.ready_cond, NULL);
    pthread_mutex_init(&dec.ready_mutex, NULL);
    pthread_create(&dec.thread, NULL, wsprDecoder, NULL);

    // Main loop : Wait, read, decode
    while (!rx_state.exit_flag) {
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
        sprintf(dec_options.date,"%02d%02d%02d", gtm->tm_year - 100, gtm->tm_mon + 1, gtm->tm_mday);
        sprintf(dec_options.uttime,"%02d%02d", gtm->tm_hour, gtm->tm_min);
        dec_options.freq = rx_options.dialfreq;

        // Start to store the samples
        initSampleStorage();

        while( (airspy_is_streaming(device) == AIRSPY_TRUE) && 
               (rx_state.exit_flag == false) && (rx_state.record_flag == true) ) {
            sleep(1);
        }
    }

    result = airspy_stop_rx(device);
    if( result != AIRSPY_SUCCESS ) {
        printf("airspy_stop_rx() failed: %s (%d)\n", airspy_error_name(result), result);
    }

    if(device != NULL) {
        result = airspy_close(device);
        if( result != AIRSPY_SUCCESS ) {
            printf("airspy_close() failed: %s (%d)\n", airspy_error_name(result), result);
        }
        airspy_exit();
    }

    printf("Bye!\n");

    /* TEST Free the thread join it */
    pthread_mutex_lock(&dec.ready_mutex); 
    pthread_cond_signal(&dec.ready_cond); 
    pthread_mutex_unlock(&dec.ready_mutex);
    pthread_join(dec.thread, NULL);

    // Destroy the lock/cond/thread
    pthread_rwlock_destroy(&dec.rw);
    pthread_cond_destroy(&dec.ready_cond);
    pthread_mutex_destroy(&dec.ready_mutex);
    pthread_exit(NULL);  

    return exit_code;
}
