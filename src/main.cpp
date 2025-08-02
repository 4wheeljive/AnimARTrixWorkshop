
#include <Arduino.h>
#include <stdio.h>
#include <string>

#define FL_ANIMARTRIX_USES_FAST_MATH 1

#include <FastLED.h>
#include "fl/slice.h"
#include "fx/fx_engine.h"

#include "myAnimartrix.hpp"

#include <FS.h>
#include "LittleFS.h"
#define FORMAT_LITTLEFS_IF_FAILED true 

//*********************************************

#define BIG_BOARD
//#undef BIG_BOARD

#define CUSTOM_MAP
//#undef CUSTOM_MAP

#define FIRST_ANIMATION PATTERN
#define SECONDS_PER_ANIMATION 10

#define DATA_PIN_1 2

//*********************************************

#ifdef BIG_BOARD 
    #define DATA_PIN_2 3
    #define DATA_PIN_3 4
    #define HEIGHT 32 
    #define WIDTH 48
    #define NUM_SEGMENTS 3
    #define NUM_LEDS_PER_SEGMENT 512
#else 
    #define HEIGHT 24 
    #define WIDTH 24
    #define NUM_SEGMENTS 1
    #define NUM_LEDS_PER_SEGMENT 576
#endif

//*********************************************

#include "bleControl.h"

#ifndef BIG_BOARD
    //#include <matrixMap_22x22.h>
    #include <matrixMap_24x24.h>
#else
    #include <matrixMap_32x48_3pin.h>    
#endif

//*********************************************

#define NUM_LEDS ( WIDTH * HEIGHT )

CRGB leds[NUM_LEDS];

using namespace fl;

// MAPPINGS **********************************************************************************
  
#ifdef CUSTOM_MAP

    uint16_t ledNum = 0;

    extern const uint16_t loc2indProgByRow[HEIGHT][WIDTH] PROGMEM;

    uint16_t myXYFunction(uint16_t x, uint16_t y, uint16_t width, uint16_t height) {
        width = WIDTH;
        height = HEIGHT;
        if (x >= width || y >= height) return 0;
        ledNum = loc2indProgByRow[y][x];
        return ledNum;
    }

    uint16_t myXYFunction(uint16_t x, uint16_t y, uint16_t width, uint16_t height);

    XYMap myXYmap = XYMap::constructWithUserFunction(WIDTH, HEIGHT, myXYFunction);

#else    
    
    XYMap myXYmap(WIDTH, HEIGHT, true); 

#endif


//************************************************************************************************************

Animartrix myAnimartrix(myXYmap, FIRST_ANIMATION);
FxEngine fxEngine(NUM_LEDS);

//**********************************************************************************************

void setColorOrder(int value) {
    switch(value) {
        case 0: value = RGB; break;
        case 1: value = RBG; break;
        case 2: value = GRB; break;
        case 3: value = GBR; break;
        case 4: value = BRG; break;
        case 5: value = BGR; break;
    }
    myAnimartrix.setColorOrder(static_cast<EOrder>(value));
}

//**********************************************************************************************

void setup() {

    Serial.begin(115200);

    FastLED.addLeds<WS2812B, DATA_PIN_1, GRB>(leds, 0, NUM_LEDS_PER_SEGMENT)
        .setCorrection(TypicalLEDStrip);

    #ifdef DATA_PIN_2
        FastLED.addLeds<WS2812B, DATA_PIN_2, GRB>(leds, NUM_LEDS_PER_SEGMENT, NUM_LEDS_PER_SEGMENT)
        .setCorrection(TypicalLEDStrip);
    #endif
    
    #ifdef DATA_PIN_3
    FastLED.addLeds<WS2812B, DATA_PIN_3, GRB>(leds, NUM_LEDS_PER_SEGMENT * 2, NUM_LEDS_PER_SEGMENT)
        .setCorrection(TypicalLEDStrip);
    #endif

    fxEngine.addFx(myAnimartrix);

    bleSetup();
    
    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS mount failed!");
        return;
    }
    Serial.println("LittleFS mounted successfully.");   

 }

//************************************************************************************************************

void loop() {
    
    if (!pauseAnimation) {
    
        if (!displayOn){
            FastLED.clear();
        }

        else {

            FastLED.setBrightness(cBright);
            fxEngine.setSpeed(1);
          
            static auto lastColorOrder = -1;
            if (cColOrd != lastColorOrder) {
                setColorOrder(cColOrd);
                lastColorOrder = cColOrd;
            } 

            static auto lastFxIndex = -1;
            if (cFxIndex != lastFxIndex) {
                lastFxIndex = cFxIndex;
                myAnimartrix.fxSet(cFxIndex);
            }
            
            fxEngine.draw(millis(), leds);
            
            /*
            if (rotateAnimations) {
                EVERY_N_SECONDS (SECONDS_PER_ANIMATION) { 
                    fxIndex += 1 % (NUM_ANIMATIONS - 1);
                    animationc(fxIndex);
                }
            }
            */
        }
        
        FastLED.show();
    }

    // upon BLE disconnect
    if (!deviceConnected && wasConnected) {
        if (debug) {Serial.println("Device disconnected.");}
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising();
        if (debug) {Serial.println("Start advertising");}
        wasConnected = false;
    }

}

//************************************************************************************************************
