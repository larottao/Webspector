 /********************************************************************************************************************************************************
 *                                                                                                                                                       *
 *  Project:         Webspector - WebServer based Spectrum Analyzer                                                                                      *
 *  Target Platform: ESP32                                                                                                                               *
 *                                                                                                                                                       * 
 *  Version: 1.0                                                                                                                                         *
 *  Hardware setup: See github                                                                                                                           *
 *                                                                                                                                                       *
 *                                                                                                                                                       * 
 *  Mark Donners                                                                                                                                         *
 *  The Electronic Engineer                                                                                                                              *
 *  Website:   www.theelectronicengineer.nl                                                                                                              *
 *  facebook:  https://www.facebook.com/TheelectronicEngineer                                                                                            *
 *  youtube:   https://www.youtube.com/channel/UCm5wy-2RoXGjG2F9wpDFF3w                                                                                  *
 *  github:    https://github.com/donnersm                                                                                                               *
 *                                                                                                                                                       *  
 ********************************************************************************************************************************************************/
 

#pragma once

#define MODE_BUTTON_PIN     15
#define GAIN_DAMPEN         2                       // Higher values cause auto gain to react more slowly
int NoiseTresshold =        1000;                   // this will effect the upper bands most.

// Other stuff don't change
#define ARRAYSIZE(a)    (sizeof(a)/sizeof(a[0]))
#define ADC_INPUT ADC1_CHANNEL_0
uint16_t offset = (int)ADC_INPUT * 0x1000 + 0xFFF;
double vReal[SAMPLEBLOCK];
double vImag[SAMPLEBLOCK];
int16_t samples[SAMPLEBLOCK];
arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
byte peak[65];
int oldBarHeights[65];
volatile float FreqBins[65];
volatile float FreqBinsOD[65];
volatile float FreqBinsNW[65];
