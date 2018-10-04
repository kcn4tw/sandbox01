/*
 ============================================================================
 Name        : bf2.c
 Author      : 
 Version     :
 Copyright   : Your copyright notice
 Description : Hello World in C, Ansi-style
 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#include <sys/types.h>
#include <sys/stat.h>

//#include "nf_math_fun.h"

#define DEBUG_SIMUL(x) {printf x;}
// #define DBG_LOG(x)

#define SPEED_OF_SOUND      (343.3f) /* unit of m */

#define MAX_NUM_MICS        (4)
#define NUM_MICS            (2)

#define MAX_FIR_INPUT_LEN   512
#define MAX_FLT_LEN         129 /* maximum length of filter than can be handled */
#define BUFFER_LEN          (MAX_FLT_LEN - 1 + MAX_FIR_INPUT_LEN) /* buffer to hold all of the input samples */

#define DAS_FLT_LEN         (65)
#define DAB_FLT_LEN         (127) /* blocking NLMS filter length */


typedef struct {
    int     len;
    float   w[MAX_FLT_LEN];
    float   x[BUFFER_LEN];
} T_FILTER;

typedef struct {
    int     num_mics;
    float   spacing; // unit: cm
    float   doa; // DOA, steering direction.
    float   fs;
    float   delay[NUM_MICS];
    int16_t mic[NUM_MICS][BUFFER_LEN];
    int16_t mic_d[NUM_MICS][BUFFER_LEN];
    int16_t das[BUFFER_LEN];
    float   f_mic[NUM_MICS][BUFFER_LEN];;
    float   f_mic_d[NUM_MICS][BUFFER_LEN];
    float   f_das[BUFFER_LEN];



    int16_t blocking[NUM_MICS-1][BUFFER_LEN];
    int16_t dab[BUFFER_LEN];
    int16_t beam_output[BUFFER_LEN];
    T_FILTER   fir_das[NUM_MICS];
    T_FILTER   fir_dab_nlms[NUM_MICS-1];
    float wDelay[NUM_MICS][DAS_FLT_LEN]; // fixed delay filter
    float v_gB[NUM_MICS][DAB_FLT_LEN]; // blocking adaptive filter
} T_BEAM;


T_BEAM Beam;


// fir filter init
static void fFirInit( T_FILTER *hdl , float *coeffs, int flt_len);

// the FIR filter function
static void f_Fir( T_FILTER *hdl, float *input, float *output, int length, int filterLength );

static void f_delay_filter(float delay, int filterLength, float *pDelayFilter, int dump);


/*
    function implementation.
*/

static void i2float(int16_t *x, float *y, int len)
{
    int i;
    for (i=0;i<len;i++) {
        y[i] = (float) x[i];
    }
}

static void float2i(float *x, int16_t *y, int len)
{
    int i;
    for (i=0;i<len;i++) {
        y[i] = (int16_t) x[i];
    }
}

static void f_delay_filter(float delay, int filterLength, float *GD, int dump)
{
    //double delay = 0.25;               // Fractional delay amount
    //int filterLength = 11;             // Number of FIR filter taps (should be odd)
    int centreTap = filterLength / 2;  // Position of centre FIR tap
    int t;
    double alpha = 0.54f;
    double epsilon = 1e-6;
    double x, sinc, window, tapWeight;

    if (fabs(delay) < epsilon) {
        delay = epsilon;
    }

    for (t=0 ; t<filterLength ; t++)
    {
        // Calculated shifted x position
        x = t - delay;

        // Calculate sinc function value
        sinc = sin(M_PI * (x-centreTap)) / (M_PI * (x-centreTap));

        // Calculate (Hamming) windowing function value
        window = alpha - (1.0f-alpha) * cos(2.0 * M_PI * (x+0.5) / filterLength);

        // Calculate tap weight
        tapWeight = window * sinc;

        // Output information
        GD[t] = (tapWeight*1.0f);
    }

    if (dump != 0) {
        FILE *fp = NULL;
        fp = fopen("gd.raw", "wb");
        fwrite( GD, sizeof(short), DAS_FLT_LEN, fp);
        fclose(fp);
    }

}


static void Init_beamformer(T_BEAM *hdl, float L, int fs, float d, float doa) {

    int i;
    float delay = 0.0f;
    float d_to_center = 0.0f;

    //DEBUG_SIMUL(("TRACE: %s [%d]\n", __FUNCTION__, __LINE__));

    hdl->num_mics = L;
    hdl->spacing = d;
    hdl->doa = doa;
    hdl->fs = (float) fs; // default values

    // reference:  http://www.labbookpages.co.uk/audio/beamforming/fractionalDelay.html

    for (i=0;i<hdl->num_mics;i++) {

        d_to_center = ((float)i-((float)hdl->num_mics-1.0f)/2.0f)*hdl->spacing;

        hdl->delay[i] = d_to_center * sin(doa*M_PI/180.f) * hdl->fs / SPEED_OF_SOUND;

        /* calculate delay frantional filter */
        f_delay_filter(hdl->delay[i], DAS_FLT_LEN, &hdl->wDelay[i][0], 0);

        fFirInit( &hdl->fir_das[i] , &hdl->wDelay[i][0], DAS_FLT_LEN);

        DEBUG_SIMUL(("delay = %f\n", hdl->delay[i]));
    }
}

/*
def gB_cal(gB,u,blocked_signal, beamformer_out):

    BLOCK_PWR = np.sum(np.multiply(blocked_signal, blocked_signal))

    if BLOCK_PWR == 0:
        gB_out = gB
    else:
        gB_out = gB + u*(blocked_signal/(BLOCK_PWR))*beamformer_out;

    return gB_out
*/
static void f_nlms(T_FILTER *p, float *x, float *y, float *des, int xN)
{
    int i;
    int N = p->len;
    float e[N];
    float u = 0.001; /* Q15, 0.001 */
    float power = 0.0f;

    for (i=0;i<xN, i++) {
    	power += y[i]*y[i];
    }



    // void firFixed( T_FIR *hdl, int16_t *input, int16_t *output, int length, int filterLength );
    f_Fir( p, x, y, xN, p->len);

    for (i=0;i<xN;i++) {
        e[i] = des[i] - y[i];
    }

}


// https://sestevenson.wordpress.com/implementation-of-fir-filtering-in-c-part-3/

// FIR init
static void fFirInit( T_FILTER *p , float *coeffs, int flt_len)
{
    int i;

    assert(flt_len<MAX_FLT_LEN);

    p->len = flt_len;

    memset( &p->x[0], 0, sizeof( p->x ) );

    for (i=0;i<flt_len;i++) {
        p->w[i] = coeffs[i];
    }
}

// the FIR filter function
static void f_Fir( T_FILTER *hdl, float *input, float *output, int length, int filterLength )
{

    int32_t acc;     // accumulator for MACs
    float *coeffp; // pointer to coefficients
    float *inputp; // pointer to input samples
    int n;
    int k;

    // put the new samples at the high end of the buffer
    memcpy( &hdl->x[filterLength - 1], input, length * sizeof(float) );

    // apply the filter to each input sample
    for ( n = 0; n < length; n++ ) {
        // calculate output n
        coeffp = hdl->w;
        inputp = &hdl->x[filterLength - 1 + n];
        acc = 0;
        for ( k = 0; k < filterLength; k++ ) {
            acc += (*coeffp++) * (*inputp--);
        }
        output[n] = acc;
    }

    // shift input samples back in time for next time
    memmove( &hdl->x[0], &hdl->x[length], (filterLength - 1) * sizeof(float) );

}

int main(int argc, char **argv)
{
    char fn[128];
    FILE *fp_mic_0 = NULL;
    FILE *fp_mic_d = NULL;
    FILE *fp_beam_out = NULL;
    int size;
    int SAMPLE_RATE = 16000;
    int i, sCount;

    DEBUG_SIMUL(("TRACE: %s [%d]\n", __FUNCTION__, __LINE__));
    DEBUG_SIMUL(("size of beam = %ld\n", sizeof(Beam)));


    while (*argv) {
        if (strcmp(*argv, "-r") == 0) {
            argv++;
            if (*argv)
                SAMPLE_RATE = atoi(*argv);
        }

        if (*argv)
            argv++;
    }

    Init_beamformer(&Beam, NUM_MICS, SAMPLE_RATE, 0.1f, 45.01f);


    snprintf(fn, 64, "./beam_mic_0_%d.raw", SAMPLE_RATE);

    fp_mic_0 = fopen(fn, "rb");
    if (!fp_mic_0) {
        DEBUG_SIMUL(("can not open %s\n", fn));
        return -1;
    }

    snprintf(fn, 64, "./beam_mic_d_%d.raw", SAMPLE_RATE);
    fp_mic_d = fopen(fn, "rb");
    if (!fp_mic_d) {
        DEBUG_SIMUL(("can not open %s\n", fn));
        return -1;
    }

    snprintf( fn, 128, "./beam_mic_out_%d.raw", SAMPLE_RATE);
    fp_beam_out = fopen(fn, "wb");

    // process all of the samples
    do {

        sCount += DAS_FLT_LEN;

        // read samples from file
        size = fread( &Beam.mic[0][0], sizeof(int16_t), DAS_FLT_LEN, fp_mic_0 );
        i2float(&Beam.mic[0][0], &Beam.f_mic[0][0], size);

        // apply each filter
        f_Fir( &Beam.fir_das[0], &Beam.f_mic[0][0], &Beam.f_mic_d[0][0], size, DAS_FLT_LEN);

        // read samples from file
        size = fread( &Beam.mic[1][0], sizeof(int16_t), DAS_FLT_LEN, fp_mic_d );
        i2float(&Beam.mic[1][0], &Beam.f_mic[1][0], size);

        f_Fir( &Beam.fir_das[1], &Beam.f_mic[1][0], &Beam.f_mic_d[1][0], size, DAS_FLT_LEN);

        /* delay and sum calculation. */
        for (i=0;i<size;i++) {
            Beam.f_das[i] = 0.5f * (Beam.f_mic_d[0][i]+Beam.f_mic_d[1][i]);
        }

        float2i(Beam.f_das, Beam.das, size);
        fwrite( Beam.das, sizeof(short int),DAS_FLT_LEN, fp_beam_out);

    } while ( size != 0 );

    DEBUG_SIMUL(("sample time = %f, fs = %f, finished.....\n", (float)sCount/Beam.fs, Beam.fs));
    return EXIT_SUCCESS;
}




