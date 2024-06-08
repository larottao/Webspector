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
 
#define VERSION     "V1.0"
//general libraries
#include <arduinoFFT.h>                                 //library for FFT analysis, Enrique Condes 1.5.6
#include <EasyButton.h>                                 //library for handling buttons 2.0.3

//included files
#include "I2SPLUGIN.h"                                  //Setting up the ADC for I2S interface ( very fast readout)
#include "FFT.h"                                        //some things for selecting the correct arrays for each number of bands
#include "Settings.h"                                   // your general settings

#include <MD_MAX72xx.h>


int numBands = 32;                                      // Default number of bands. change it by pressing the mode button

// Pin definitions for LED matrix
#define DATA_IN 23
#define CLK_PIN 18
#define CS_PIN 5
#define MAX_DEVICES 4

// Maximum value for LED scaling
float maxValue = 10.0;


//*************Button setup ******************************************************************************************************************************
EasyButton ModeBut(MODE_BUTTON_PIN);                    //defining the button
// Mode button 1 short press
// will result in changing the number of bands
void onPressed() {
  Serial.println("Mode Button has been pressed!");
  if (numBands == 8)numBands = 16;
  else if (numBands == 16)numBands = 24;
  else if (numBands == 24)numBands = 32;
  else if (numBands == 32)numBands = 64;
  else if (numBands == 64)numBands = 8;
  SetNumberofBands(numBands);
  Serial.printf("New number of bands=%d\n", numBands);
}
//*************Button setup end***************************************************************************************************************************

// LED matrix object
MD_MAX72XX mx = MD_MAX72XX(MD_MAX72XX::FC16_HW, DATA_IN, CLK_PIN, CS_PIN, MAX_DEVICES);

void setup() {
  delay(500);
  Serial.begin(115200);
  Serial.println("Setting up Audio Input I2S");
  setupI2S();
  delay(100);
  i2s_adc_enable(I2S_NUM_0);
  Serial.println("Audio input setup completed");
  ModeBut.onPressed(onPressed);

  SetNumberofBands(numBands);

  mx.begin();
  mx.control(MD_MAX72XX::INTENSITY, 8); // Set intensity level
}

void loop() {
  size_t bytesRead = 0;
  int TempADC = 0;
  ModeBut.read();

  //############ Step 1: read samples from the I2S Buffer ##################
  i2s_read(I2S_PORT,
           (void*)samples,
           sizeof(samples),
           &bytesRead,   // workaround This is the actual buffer size last half will be empty but why?
           portMAX_DELAY); // no timeout

  if (bytesRead != sizeof(samples)) {
    Serial.printf("Could only read %u bytes of %u in FillBufferI2S()\n", bytesRead, sizeof(samples));
  }

  //############ Step 2: compensate for Channel number and offset, safe all to vReal Array   ############
  for (uint16_t i = 0; i < ARRAYSIZE(samples); i++) {
    vReal[i] = offset - samples[i];
    vImag[i] = 0.0; //Imaginary part must be zeroed in case of looping to avoid wrong calculations and overflows
  }


  //############ Step 3: Do FFT on the VReal array  ############
  // compute FFT
  FFT.DCRemoval();
  FFT.Windowing(vReal, SAMPLEBLOCK, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLEBLOCK, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLEBLOCK);
  FFT.MajorPeak(vReal, SAMPLEBLOCK, samplingFrequency);
  for (int i = 0; i < numBands; i++) {
    FreqBins[i] = 0;
  }
  //############ Step 4: Fill the frequency bins with the FFT Samples ############
  for (int i = 2; i < SAMPLEBLOCK / 2; i++) {
    if (vReal[i] > NoiseTresshold) {
      int freq = BucketFrequency(i);
      int iBand = 0;
      while (iBand < numBands) {
        if (freq < BandCutoffTable[iBand])break;
        iBand++;
      }
      if (iBand > numBands)iBand = numBands;
      FreqBins[iBand] += vReal[i];
    }
  }


  //############ Step 5: Averaging and making it all fit on screen
  static float lastAllBandsPeak = 0.0f;
  float allBandsPeak = 0;
  for (int i = 0; i < numBands; i++) {
    if (FreqBins[i] > allBandsPeak) {
      allBandsPeak = FreqBins[i];
    }
  }
  if (allBandsPeak < 1)allBandsPeak = 1;
  allBandsPeak = max(allBandsPeak, ((lastAllBandsPeak * (GAIN_DAMPEN - 1)) + allBandsPeak) / GAIN_DAMPEN); // Dampen rate of change a little bit on way down
  lastAllBandsPeak = allBandsPeak;
  if (allBandsPeak < 80000)allBandsPeak = 80000;
  for (int i = 0; i < numBands; i++)FreqBins[i] /= (allBandsPeak * 1.0f);

  SendData(); // Print the data to serial

  displayFFTData(); // Output the data to the LED matrix

} // loop end


// Return the frequency corresponding to the Nth sample bucket.  Skips the first two
// buckets which are overall amplitude and something else.
int BucketFrequency(int iBucket) {
  if (iBucket <= 1)return 0;
  int iOffset = iBucket - 2;
  return iOffset * (samplingFrequency / 2) / (SAMPLEBLOCK / 2);
}

void SendData() {
  String json = "[";
  for (int i = 0; i < numBands; i++) {
    if (i > 0) {
      json += ", ";
    }
    json += "{\"bin\":";
    json += "\"" + labels[i] + "\"";
    json += ", \"value\":";
    json += String(FreqBins[i]);
    json += "}";
  }
  json += "]";
  Serial.println(json); // Print JSON to serial
}

void displayFFTData() {
  mx.clear();
  for (int i = 0; i < numBands; i++) {
    int numLEDs = map(FreqBins[i], 0, maxValue, 0, 8); // Scale to 0-8 LEDs
    for (int j = 0; j < numLEDs; j++) {
      mx.setPoint(j, i, true);
    }
  }
  mx.update();
}
