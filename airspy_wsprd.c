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
 - BUG : bit packing not working
 - clean/fix samplerate selection
 - clean/fix serial number section
 - multispot report in one post
 - type fix (uint32_t etc..)
*/


#define SIGNAL_LENGHT      116
#define SIGNAL_SAMPLE_RATE 375


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
    uint32_t sigLenght = transfer->sample_count;

    static uint32_t decimationIndex=0;
    
    /* CIC buffers */
    static int32_t  Ix1,Ix2,Qx1,Qx2;
    static int32_t  Iy1,It1y,It1z,Qy1,Qt1y,Qt1z;
    static int32_t  Iy2,It2y,It2z,Qy2,Qt2y,Qt2z;

    /* FIR compensation filter buffers */
    static float    firI[32], firQ[32];

    /* FIR compensation filter coefs
       Using : Octave/MATLAB code for generating compensation FIR coefficients
       URL : https://github.com/WestCoastDSP/CIC_Octave_Matlab
     */
    const static float zCoef[33] = {
        -0.0027772683, -0.0005058826,  0.0049745750, -0.0034059318,
        -0.0077557814,  0.0139375423,  0.0039896935, -0.0299394142,
         0.0162250643,  0.0405130860, -0.0580746013, -0.0272104968,
         0.1183705475, -0.0306029022, -0.2011241667,  0.1615898423,
         0.5000000000,
         0.1615898423, -0.2011241667, -0.0306029022,  0.1183705475,
        -0.0272104968, -0.0580746013,  0.0405130860,  0.0162250643,
        -0.0299394142,  0.0039896935,  0.0139375423, -0.0077557814,
        -0.0034059318,  0.0049745750, -0.0005058826, -0.0027772683
    };
    float Isum,Qsum;

    /* Convert unsigned signal to signed, without bit shift */
    for(int32_t i=0; i<sigLenght; i++)
        sigIn[i] = (sigIn[i] & 0xFFF) - 2048;

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
    for (uint32_t i=0; i<sigLenght; i+=8) {
        tmp = sigIn[i+3];
        sigIn[i+3] = -sigIn[i+2];
        sigIn[i+2] = tmp;

        sigIn[i+4] = -sigIn[i+4];
        sigIn[i+5] = -sigIn[i+5];

        tmp = sigIn[i+6];
        sigIn[i+6] = -sigIn[i+7];
        sigIn[i+7] = tmp;
    }

    /* CIC decimator (N=2)
       (could be not perfect in time for some sampling rate.
       Ex: AirSpy vs AirSpy Mini, but works fine in practice)
       Info: * Understanding CIC Compensation Filters
               https://www.altera.com/en_US/pdfs/literature/an/an455.pdf
             * Understanding cascaded integrator-comb filters
               http://www.embedded.com/design/configurable-systems/4006446/Understanding-cascaded-integrator-comb-filters
    */
    for(int32_t i=0; i<sigLenght/2; i++) {
        /* Integrator stages (N=2) */
        Ix1 += (int32_t)sigIn[i*2];
        Qx1 += (int32_t)sigIn[i*2+1];
        Ix2 += Ix1;
        Qx2 += Qx1;

        /* Decimation R=n (ex. rx_options.downsampling=6667) */
        decimationIndex++;
        if (decimationIndex < rx_options.downsampling) {
            continue;
        }

        /* 1st Comb */
        Iy1  = Ix2 - It1z;
        It1z = It1y;
        It1y = Ix2;
        Qy1  = Qx2 - Qt1z;
        Qt1z = Qt1y;
        Qt1y = Qx2;

        /* 2nd Comd */
        Iy2  = Iy1 - It2z;
        It2z = It2y;
        It2y = Iy1;
        Qy2  = Qy1 - Qt2z;
        Qt2z = Qt2y;
        Qt2y = Qy1;
        // FIXME/TODO : some optimisition here

        /* FIR compensation filter */
        Isum=0.0, Qsum=0.0;
        for (uint32_t j=0; j<32; j++) {
            Isum += firI[j]*zCoef[j];
            Qsum += firQ[j]*zCoef[j];
            if (j<31) {
                firI[j] = firI[j+1];
                firQ[j] = firQ[j+1];
            }
        }
        firI[31] = (float)Iy2;
        firQ[31] = (float)Qy2;
        Isum += firI[31]*zCoef[32];
        Qsum += firQ[31]*zCoef[32];

        /* Save the result in the buffer */
        if (rx_state.iqIndex < (SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE)) {
            /* Lock the buffer during writing */     // Overkill ?!
            pthread_rwlock_wrlock(&dec.rw);
            rx_state.iSamples[rx_state.iqIndex] = Isum;
            rx_state.qSamples[rx_state.iqIndex] = Qsum;
            pthread_rwlock_unlock(&dec.rw);
            rx_state.iqIndex++;
        } else {
            if (rx_state.decode_flag == false) {
                /* Send a signal to the other thread to start the decoding */
                pthread_mutex_lock(&dec.ready_mutex);
                pthread_cond_signal(&dec.ready_cond);
                pthread_mutex_unlock(&dec.ready_mutex);
                rx_state.decode_flag = true;
                printf("RX done! [Buffer size: %d]\n", rx_state.iqIndex);
            }
        }
        
        decimationIndex = 0;
    }
    return 0;
}


void postSpots(uint32_t n_results) {
    CURL *curl;
    CURLcode res;
    char url[256]; // FIXME, possible buffer overflow

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
    /* WSPR decoder use buffers of 45000 samples (fixed value)
       (120 sec max @ 375sps = 45000 samples)
    */
    static float iSamples[45000]={0};
    static float qSamples[45000]={0};
    static uint32_t samples_len;
    int32_t n_results=0;

    while (!rx_state.exit_flag) {
        pthread_mutex_lock(&dec.ready_mutex);
        pthread_cond_wait(&dec.ready_cond, &dec.ready_mutex);
        pthread_mutex_unlock(&dec.ready_mutex);

        if(rx_state.exit_flag)  // Abord case, final sig
            break;

        /* Lock the buffer access and make copy */
        pthread_rwlock_wrlock(&dec.rw);
        memcpy(iSamples, rx_state.iSamples, rx_state.iqIndex * sizeof(float));
        memcpy(qSamples, rx_state.qSamples, rx_state.iqIndex * sizeof(float));
        samples_len = rx_state.iqIndex;  // Overkill ?
        pthread_rwlock_unlock(&dec.rw);

        /* Date and time will be updated/overload during the search & decoding process
           Make a simple copy
        */
        memcpy(dec_options.date, rx_options.date, sizeof(rx_options.date));
        memcpy(dec_options.uttime, rx_options.uttime, sizeof(rx_options.uttime));

        /* Search & decode the signal */
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


int32_t parse_u64(char* s, uint64_t* const value) {
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
    rx_state.decode_flag = false;
    rx_state.iqIndex=0;
}


/* Default options for the decoder */
void initDecoder_options() {
    dec_options.usehashtable = 1;
    dec_options.npasses = 2;
    dec_options.subtraction = 1;
    dec_options.quickmode = 0;
}


/* Default options for the receiver */
void initrx_options() {
    rx_options.lnaGain = 3;    // DEFAULT_LNA_GAIN
    rx_options.mixerGain = 5;  // DEFAULT_MIXER_GAIN
    rx_options.vgaGain = 5;    // DEFAULT_VGA_IF_GAIN
    rx_options.bias = 0;       // No bias
    rx_options.shift = 0;
    rx_options.rate = 2500000; 
    rx_options.serialnumber = 0;  
    rx_options.packing = 0;
}


void sigint_callback_handler(int signum) {
    fprintf(stdout, "Caught signal %d\n", signum);
    rx_state.exit_flag = true;
}


void usage(void) {
    fprintf(stderr,
            "airspy_wsprd, a simple WSPR daemon for AirSpy receivers\n\n"
            "Use:\tairspy_wsprd -f frequency -c callsign -g locator [options]\n"
            "\t-f dial frequency [(,k,M) Hz], check http://wsprnet.org/ for freq.\n"
            "\t-c your callsign (12 chars max)\n"
            "\t-g your locator grid (6 chars max)\n"
            "Receiver extra options:\n"
            "\t-l LNA gain [0-14] (default: 3)\n"
            "\t-m MIXER gain [0-15] (default: 5)\n"
            "\t-v VGA gain [0-15] (default: 5)\n"
            "\t-b set Bias Tee [0-1], (default: 0 disabled)\n"
            "\t-r sampling rate [2.5M, 3M, 6M, 10M], (default: 2.5M)\n"
            "\t-p frequency correction (default: 0)\n"
            "\t-s S/N: Open device with specified 64bits serial number\n"
            "\t-k packing: Set packing for samples, \n"
            "\t   1=enabled(12bits packed), 0=disabled(default 16bits not packed)\n"
            "Decoder extra options:\n"
            "\t-H do not use (or update) the hash table\n"
            "\t-Q quick mode, doesn't dig deep for weak signals\n"
            "\t-S single pass mode, no subtraction (same as original wsprd)\n"
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

    /* RX buffer allocation */
    rx_state.iSamples=malloc(sizeof(float)*SIGNAL_LENGHT*SIGNAL_SAMPLE_RATE);
    rx_state.qSamples=malloc(sizeof(float)*SIGNAL_LENGHT*SIGNAL_SAMPLE_RATE);

    /* Stop condition setup */
    rx_state.exit_flag   = false;
    rx_state.decode_flag = false;

    if (argc <= 1)
        usage();

    while ((opt = getopt(argc, argv, "f:c:g:r:l:m:v:b:s:p:k:H:Q:S")) != -1) {
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
        case 's': // Serial number
            parse_u64(optarg, &rx_options.serialnumber);
            break;
        case 'p': // Fine frequency correction
            rx_options.shift = (int32_t)atoi(optarg);
            break;
        case 'k': // Bit packing
            rx_options.packing = (uint32_t)atoi(optarg);
            if (rx_options.packing < 0) rx_options.packing = 0;
            if (rx_options.packing > 1) rx_options.packing = 1;
            break;
        case 'H': // Decoder option, use a hastable 
            dec_options.usehashtable = 0;
        case 'Q': // Decoder option, faster
            dec_options.quickmode = 1;
        case 'S': // Decoder option, single pass mode (same as original wsprd)
            dec_options.subtraction = 0;
            dec_options.npasses = 1;
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

    /* Calcule decimation rate & frequency offset for fs/4 shift */
    rx_options.fs4 = rx_options.rate / 4;
    rx_options.downsampling = (uint32_t)round((double)rx_options.rate / 375.0);
    rx_options.realfreq = rx_options.dialfreq + rx_options.shift;

    /* Store the frequency used for the decoder */
    dec_options.freq = rx_options.dialfreq;

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

    if(rx_options.packing) {
        result = airspy_set_packing(device, 1);
        if( result != AIRSPY_SUCCESS ) {
            printf("airspy_set_packing() failed: %s (%d)\n", airspy_error_name(result), result);
            airspy_close(device);
            airspy_exit();
            return EXIT_FAILURE;
        }
    }

    result = airspy_set_sample_type(device, AIRSPY_SAMPLE_UINT16_REAL);
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

    result = airspy_set_freq(device, rx_options.realfreq + rx_options.fs4 + 1500);  // Dial + offset + 1500Hz
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

    /* Print used parameter */
    time_t rawtime;
    time ( &rawtime );
    struct tm *gtm = gmtime(&rawtime);
    printf("Starting airspy-wsprd (%04d-%02d-%02d, %02d:%02dz) -- Version 0.1\n",
           gtm->tm_year + 1900, gtm->tm_mon + 1, gtm->tm_mday, gtm->tm_hour, gtm->tm_min);
    printf("  Callsign     : %s\n", dec_options.rcall);
    printf("  Locator      : %s\n", dec_options.rloc);
    printf("  Dial freq.   : %d Hz\n", rx_options.dialfreq);
    printf("  Real freq.   : %d Hz\n", rx_options.realfreq);
    printf("  Rate         : %d Hz\n", rx_options.rate);
    printf("  Decimation   : %d\n", rx_options.downsampling);
    printf("  LNA gain     : %d dB\n", rx_options.lnaGain);
    printf("  Mixer gain   : %d dB\n", rx_options.mixerGain);
    printf("  VGA gain     : %d dB\n", rx_options.vgaGain);
    printf("  Bias         : %s\n", rx_options.bias ? "yes" : "no");
    printf("  Bits packing : %s\n", rx_options.packing ? "yes" : "no");
    printf("  S/N          : 0x%08X%08X\n", readSerial.serial_no[2], readSerial.serial_no[3]);

    /* Create a thread and stuff for separate decoding
       Info : https://computing.llnl.gov/tutorials/pthreads/
    */
    pthread_rwlock_init(&dec.rw, NULL);
    pthread_cond_init(&dec.ready_cond, NULL);
    pthread_mutex_init(&dec.ready_mutex, NULL);
    pthread_create(&dec.thread, NULL, wsprDecoder, NULL);

    /* Main loop : Wait, read, decode */
    while (!rx_state.exit_flag) {
        /* Time Sync on 2 mins */
        struct timeval lTime;
        gettimeofday(&lTime, NULL);

        uint32_t sec   = lTime.tv_sec % 120;
        uint32_t usec  = sec * 1000000 + lTime.tv_usec;
        uint32_t uwait = 120000000 - usec;
        printf("Wait for time sync (start in %d sec)\n", uwait/1000000);
        usleep(uwait);
        usleep(10000); // Adding 10ms, to be sure to reach this next minute
        printf("SYNC! RX started\n");

        /* Use the Store the date at the begin of the frame */
        time ( &rawtime );
        gtm = gmtime(&rawtime);
        sprintf(rx_options.date,"%02d%02d%02d", gtm->tm_year - 100, gtm->tm_mon + 1, gtm->tm_mday);
        sprintf(rx_options.uttime,"%02d%02d", gtm->tm_hour, gtm->tm_min);

        /* Start to store the samples */
        initSampleStorage();

        while( (airspy_is_streaming(device) == AIRSPY_TRUE) && 
               (rx_state.exit_flag == false) && 
               (rx_state.iqIndex < (SIGNAL_LENGHT * SIGNAL_SAMPLE_RATE) ) ) {
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

    /* Wait the thread join (send a signal before to terminate the job) */
    pthread_mutex_lock(&dec.ready_mutex);
    pthread_cond_signal(&dec.ready_cond);
    pthread_mutex_unlock(&dec.ready_mutex);
    pthread_join(dec.thread, NULL);

    /* Destroy the lock/cond/thread */
    pthread_rwlock_destroy(&dec.rw);
    pthread_cond_destroy(&dec.ready_cond);
    pthread_mutex_destroy(&dec.ready_mutex);
    pthread_exit(NULL);

    return exit_code;
}
