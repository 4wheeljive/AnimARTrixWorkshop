#pragma once

#include "fl/vector.h"
#include <math.h>
#include "fl/stdint.h"

#include "crgb.h"
#include "fl/force_inline.h"
#include "fl/namespace.h"
#include "fl/math.h"
#include "fl/compiler_control.h"

#include "bleControl.h"

#include <string>
#include <vector>
//#include <memory>
#include <variant>

#include <Printable.h>

#ifndef FL_ANIMARTRIX_USES_FAST_MATH
#define FL_ANIMARTRIX_USES_FAST_MATH 1
#endif

#define FL_SIN_F(x) sinf(x)
#define FL_COS_F(x) cosf(x)

#if FL_ANIMARTRIX_USES_FAST_MATH
FL_FAST_MATH_BEGIN
FL_OPTIMIZATION_LEVEL_O3_BEGIN
#endif

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

#define timeFactors 5

namespace animartrix_detail {

  FASTLED_USING_NAMESPACE

    /*
    class ParameterSet {
       public: 

        float dist; 
        float angle;
        float scale_x;
        float scale_y;
        float scale_z;
        float offset_x;
        float offset_y;
        float offset_z;
        float z;
        float low_limit;
        float high_limit;

        float* param[11];

        ParameterSet() {
            // Initialize parameter float array to point to named parameters
            param[0] = &dist;
            param[1] = &angle;
            param[2] = &scale_x;
            param[3] = &scale_y;
            param[4] = &scale_z;
            param[5] = &offset_x;
            param[6] = &offset_y;
            param[7] = &offset_z;
            param[8] = &z;
            param[9] = &low_limit;
            param[10] = &high_limit;
        }

    };
    */

    struct timingMap {
        float master_speed; // global transition speed
        float offset[timeFactors];  // offsets create initial separation between timeFactors 
        float ratio[timeFactors]; // ratios determine relationships between timeFactors 
    };

    struct modulators {
        float linear[timeFactors];      // returns 0 to FLT_MAX
        float radial[timeFactors];      // returns 0 to 2*PI
        float directional[timeFactors]; // returns -1 to 1
        float noise_angle[timeFactors]; // returns 0 to 2*PI
    };

    struct rgb {
        float red, green, blue;
    };

    static const uint8_t PERLIN_NOISE[] = {
        151, 160, 137, 91,  90,  15,  131, 13,  201, 95,  96,  53,  194, 233, 7,
        225, 140, 36,  103, 30,  69,  142, 8,   99,  37,  240, 21,  10,  23,  190,
        6,   148, 247, 120, 234, 75,  0,   26,  197, 62,  94,  252, 219, 203, 117,
        35,  11,  32,  57,  177, 33,  88,  237, 149, 56,  87,  174, 20,  125, 136,
        171, 168, 68,  175, 74,  165, 71,  134, 139, 48,  27,  166, 77,  146, 158,
        231, 83,  111, 229, 122, 60,  211, 133, 230, 220, 105, 92,  41,  55,  46,
        245, 40,  244, 102, 143, 54,  65,  25,  63,  161, 1,   216, 80,  73,  209,
        76,  132, 187, 208, 89,  18,  169, 200, 196, 135, 130, 116, 188, 159, 86,
        164, 100, 109, 198, 173, 186, 3,   64,  52,  217, 226, 250, 124, 123, 5,
        202, 38,  147, 118, 126, 255, 82,  85,  212, 207, 206, 59,  227, 47,  16,
        58,  17,  182, 189, 28,  42,  223, 183, 170, 213, 119, 248, 152, 2,   44,
        154, 163, 70,  221, 153, 101, 155, 167, 43,  172, 9,   129, 22,  39,  253,
        19,  98,  108, 110, 79,  113, 224, 232, 178, 185, 112, 104, 218, 246, 97,
        228, 251, 34,  242, 193, 238, 210, 144, 12,  191, 179, 162, 241, 81,  51,
        145, 235, 249, 14,  239, 107, 49,  192, 214, 31,  181, 199, 106, 157, 184,
        84,  204, 176, 115, 121, 50,  45,  127, 4,   150, 254, 138, 236, 205, 93,
        222, 114, 67,  29,  24,  72,  243, 141, 128, 195, 78,  66,  215, 61,  156,
        180};

    FASTLED_FORCE_INLINE uint8_t P(uint8_t x) {
        const uint8_t idx = x & 255;
        const uint8_t *ptr = PERLIN_NOISE + idx;
        return *ptr;
    }

    class ANIMartRIX {

      public:
        uint8_t num_x; // matrix width
        uint8_t num_y; // matrix height

        float center_x = (num_x / 2) - 0.5; 
        float center_y = (num_y / 2) - 0.5;

        uint32_t currentTime = 0;
        
        float radial_filter_radius = 23.0; // on 32x32, use 11 for 16x16
        float radialDimmer = 1;
        float radialFilterFalloff = 1;

        bool serpentine;
        
        float show[5] = {0,1,2,3,4};

        //ParameterSet animation;         // all animation parameters in one place (an instance of the ParameterSet class)
        timingMap timings;              // all speed settings in one place (an instance of the timingMap structure)
        modulators move;                // all timing-based movers and shifters in one place (an instance of the modulators structure)
        rgb pixel;                      // an instance of the rgb structure

        fl::HeapVector<fl::HeapVector<float>>
            polar_theta; // look-up table for polar angles
        fl::HeapVector<fl::HeapVector<float>>
            distance; // look-up table for polar distances

        ANIMartRIX() {}

        ANIMartRIX(uint8_t w, uint8_t h) { this->init(w, h); }

        virtual ~ANIMartRIX() {}

        // UTILITY FUNCTIONS ***************************************************

        virtual uint16_t xyMap(uint16_t x, uint16_t y) = 0;

        void setTime(uint32_t t) { currentTime = t; }
        uint32_t getTime() { return currentTime ? currentTime : millis(); }

        // given a static polar origin we can precalculate the polar coordinates
        void render_polar_lookup_table(float cx, float cy) {

            polar_theta.resize(num_x, fl::HeapVector<float>(num_y, 0.0f));
            distance.resize(num_x, fl::HeapVector<float>(num_y, 0.0f));

            for (int xx = 0; xx < num_x; xx++) {
                for (int yy = 0; yy < num_y; yy++) {

                    float dx = xx - cx;
                    float dy = yy - cy;

                    distance[xx][yy] = hypotf(dx, dy);
                    polar_theta[xx][yy] = atan2f(dy, dx);
                }
            }
        }

        // Noise engine **********************************************
        
        float fade(float t) { return t * t * t * (t * (t * 6 - 15) + 10); }
        float lerp(float t, float a, float b) { return a + t * (b - a); }
        float grad(int hash, float x, float y, float z) {
            int h = hash & 15;       // CONVERT LO 4 BITS OF HASH CODE 
            float u = h < 8 ? x : y, // INTO 12 GRADIENT DIRECTIONS.   
                v = h < 4                ? y
                    : h == 12 || h == 14 ? x
                                        : z;
            return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
        }

        float pnoise(float x, float y, float z) {

            int X = (int)floorf(x) & 255, // FIND UNIT CUBE THAT 
                Y = (int)floorf(y) & 255, // CONTAINS POINT.     
                Z = (int)floorf(z) & 255;
            x -= floorf(x); // FIND RELATIVE X,Y,Z 
            y -= floorf(y); // OF POINT IN CUBE.   
            z -= floorf(z);
            float u = fade(x), // COMPUTE FADE CURVES 
                v = fade(y),   // FOR EACH OF X,Y,Z.  
                w = fade(z);
            int A = P(X) + Y, AA = P(A) + Z,
                AB = P(A + 1) + Z, // HASH COORDINATES OF 
                B = P(X + 1) + Y, BA = P(B) + Z,
                BB = P(B + 1) + Z; // THE 8 CUBE CORNERS, 

            return lerp(w,
                        lerp(v,
                            lerp(u, grad(P(AA), x, y, z),       // AND ADD 
                                grad(P(BA), x - 1, y, z)),      // BLENDED 
                            lerp(u, grad(P(AB), x, y - 1, z),    // RESULTS
                                grad(P(BB), x - 1, y - 1, z))), // FROM  8 
                        lerp(v,
                            lerp(u, grad(P(AA + 1), x, y, z - 1),   // CORNERS
                                grad(P(BA + 1), x - 1, y, z - 1)), // OF CUBE 
                            lerp(u, grad(P(AB + 1), x, y - 1, z - 1),
                                grad(P(BB + 1), x - 1, y - 1, z - 1))));
        }

        //***************************************************************

        // float mapping maintaining 32 bit precision
        // we keep values with high resolution for potential later usage
        float map_float(float x, float in_min, float in_max, float out_min,
                        float out_max) {

            float result =
                (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
            if (result < out_min)
                result = out_min;
            if (result > out_max)
                result = out_max;

            return result;
        }

        float radialFilterFactor( float radius, float distance, float falloff) {
            if (distance >= radius) return 0.0f;
            float factor = 1.0f - (distance / radius);
            return powf(factor, falloff);
        }

        // OPERATING FUNCTIONS ***********************************************************
        
        void init(uint8_t w, uint8_t h) {
            //animation = ParameterSet();
            timings = timingMap();
            move = modulators();
            pixel = rgb();

            this->num_x = w;
            this->num_y = h;

            this->radial_filter_radius = std::min(w,h) * 0.65;
        
            // precalculate all polar coordinates; polar origin is set to matrix center
            render_polar_lookup_table(
                (num_x / 2) - 0.5,
                (num_y / 2) - 0.5);  

        } // init()

        void calculate_modulators(timingMap &timings) {

            double runtime = getTime() * timings.master_speed * cSpeed; 

            for (int i = 0; i < timeFactors; i++) {

                // continously rising value, returns 0 to max_float
                move.linear[i] = 
                    (runtime + timings.offset[i]) * timings.ratio[i];
                if (debug){
                    Serial.print("move.linear[");
                    Serial.print(i);
                    Serial.print("] = ");
                    Serial.println(move.linear[i]);
                }

                // angle values for continous rotation, returns 0 to 2 * PI
                move.radial[i] = 
                    fmodf(move.linear[i], 2 * PI); 
                if (debug){
                    Serial.print("move.radial[");
                    Serial.print(i);
                    Serial.print("] = ");
                    Serial.println(move.radial[i]);
                }

                // directional values, returns -1 to 1
                move.directional[i] = 
                    FL_SIN_F(move.radial[i]); 
                if (debug){
                    Serial.print("move.directional[");
                    Serial.print(i);
                    Serial.print("] = ");
                    Serial.println(move.directional[i]);
                }

                // noise-based angle adjustment value, returns 0 to 2 * PI
                move.noise_angle[i] =
                    PI * (1 + pnoise(move.linear[i], 0, 0));
                if (debug){
                    Serial.print("move.noise_angle[");
                    Serial.print(i);
                    Serial.print("] = ");
                    Serial.println(move.noise_angle[i]);
                }
            }
        }

        //float render_value(ParameterSet &animation) 
        float render_value(float dist, float angle, float scale_x, float scale_y, float scale_z,
                   float offset_x, float offset_y, float offset_z, float z,
                   float low_limit, float high_limit) {

            // convert polar coordinates back to cartesian ones
            float newx = (offset_x + center_x - (FL_COS_F(angle) * dist)) * scale_x;
            float newy = (offset_y + center_y - (FL_SIN_F(angle) * dist)) * scale_y;
            float newz = (offset_z + z) * scale_z;

            if (debug) {
                Serial.print("newx: ");
                Serial.println(newx);
                Serial.print("newy: ");
                Serial.println(newy);
                Serial.print("newz: ");
                Serial.println(newz);
            }

            // render noisevalue at this new cartesian point
            float raw_noise_field_value = pnoise(newx, newy, newz);

            if (raw_noise_field_value < low_limit)
                raw_noise_field_value = low_limit;
            if (raw_noise_field_value > high_limit)
                raw_noise_field_value = high_limit;

            float scaled_noise_value =
                map_float(raw_noise_field_value, low_limit,
                        high_limit, 0, 255);

            return scaled_noise_value;
        } // render_value

        rgb rgb_sanity_check(rgb &pixel) {

            // Can never be negative colour
            if (pixel.red < 0)
                pixel.red = 0;
            if (pixel.green < 0)
                pixel.green = 0;
            if (pixel.blue < 0)
                pixel.blue = 0;

            // discard everything above the valid 8 bit colordepth 0-255 range
            if (pixel.red > 255)
                pixel.red = 255;
            if (pixel.green > 255)
                pixel.green = 255;
            if (pixel.blue > 255)
                pixel.blue = 255;

            return pixel;
        }   

        virtual void setPixelColorInternal(int x, int y, rgb pixel) = 0;


        //********************************************************************************************************************
        // COMPONENT ARCHITECTURE ********************************************************************************************

        //**************************************************************************************** 
        // Element Class 
        // Elements are instructions for how to calculate a Parameter

        // Elements include an operator, a type, a optional reference, an optional value, and an optional index
        // reference is used to identify which variable or variable array to reference
        // value is used to provide a directlty-entered float value
        // index is used for the array variables (i.e., modulators)  


        enum class ElementOperator {
            ADD = 0,      // +
            SUBTRACT, // -
            MULTIPLY,  // *
            DIVIDE      // /
        };

        enum class ElementType {
            direct = 0,
            variable,
            array
        };

        enum class ElementReference {
            null = 0,
            moveLinear,
            moveRadial,
            moveDirectional,
            moveNoiseAngle,
            pixelDistance,
            pixelAngle,
            cZoom,
            cTwist
        };

        class Element { 
          
          public:  
          
            ElementOperator op = ElementOperator::ADD;  // Default to addition
            ElementType type = ElementType::direct; // Default to direct
            ElementReference ref = ElementReference::moveLinear; // default to moveLinear
            float value = 0.0f; // default to 0
            uint8_t index = 0; // Optional index for array variables 
            
            // Constructor
            Element(    ElementOperator operation = ElementOperator::ADD, 
                        ElementType type = ElementType::direct,
                        ElementReference ref = ElementReference::moveLinear,
                        float value = 0.0f,
                        uint8_t index = 0
                    )
                        : op(operation), type(type), ref(ref), value(value), index(index)  {}



            /*virtual size_t printTo(Print& p) const {
                size_t bytesWritten = 0;
                bytesWritten += p.print("Operator: ");
                bytesWritten += p.print(op);
                bytesWritten += p.print(", Value: ");
                bytesWritten += p.println(_value);
                return bytesWritten;
            }*/


        };// class Element


        //**************************************************************************************** 
        // PARAMETER CLASS - each parameter returns a float
       
        class Parameter {

          public:

            std::vector<Element> element;

            // Default constructor - defaults to 0.0f
            Parameter() {} 
            //    : element(0.0f, ElementOperator::ADD) {}

            // Constructor FOR one or more elements
            Parameter(const std::vector<Element>& adjs)
                : element(adjs) {}

        }; // Parameter class
                
        //**************************************************************************************** 
        // LAYER CLASS

        class Layer {
                
            public:
        
            bool layerActive = false;

            // Named parameters (public for direct access)
            Parameter distance_xy;
            Parameter angle;
            Parameter scale_x;
            Parameter scale_y;
            Parameter scale_z;
            Parameter offset_x;
            Parameter offset_y;
            Parameter offset_z;
            Parameter z;
            Parameter low_limit;
            Parameter high_limit;
            
            // Array access to the same parameters
            Parameter* parameter[11];

            // Vector to store/serve current Parameter values
            std::vector<float> current = {0.0f, 0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};

            // Constructor
            Layer() {
                // Initialize parameter array to point to named parameters
                parameter[0] = &distance_xy;
                parameter[1] = &angle;
                parameter[2] = &scale_x;
                parameter[3] = &scale_y;
                parameter[4] = &scale_z;
                parameter[5] = &offset_x;
                parameter[6] = &offset_y;
                parameter[7] = &offset_z;
                parameter[8] = &z;
                parameter[9] = &low_limit;
                parameter[10] = &high_limit;
                        
            }

        }; // Layer class

        //************************************************************

        Layer layer1;
        Layer layer2;
        Layer layer3;
        Layer layer4;
        Layer layer5;
                
        Layer* layer[5] = {&layer1, &layer2, &layer3, &layer4, &layer5};

        
        //************************************************************
        
        //void createLayer(uint8_t i, uint8_t x, uint8_t y, float pixelDistance, float pixelAngle) {
        void createLayers() {

            for (uint8_t x = 0; x < num_x; x++) {
                for (uint8_t y = 0; y < num_y; y++) {
                   
                    float pixelDistance = distance[x][y];
                    float pixelAngle = polar_theta[x][y];

                    for (uint8_t i = 0; i < 5 ; i++) {
                        
                        if(layer[i]->layerActive) {
                            
                            // calculate value for each parameter
                            for (uint8_t j = 0; j < 11; j++) { 
                            
                                float result = 0.0f;
                        
                                // apply each parameter element
                                for (uint8_t k = 0; k < layer[i]->parameter[j]->element.size(); k++) { 
                                    
                                    float adjVal = 0.0f;

                                    // get a value or reference variable to apply
                                    switch (layer[i]->parameter[j]->element[k].type) { 
                                    
                                        case ElementType::direct:
                                            adjVal = layer[i]->parameter[j]->element[k].value;
                                            break;
                                        
                                        case ElementType::variable:

                                            switch (layer[i]->parameter[j]->element[k].ref) {

                                                case ElementReference::pixelDistance:
                                                    adjVal = pixelDistance;
                                                    break;

                                                case ElementReference::pixelAngle:
                                                    adjVal = pixelAngle;
                                                    break;

                                                case ElementReference::cZoom:
                                                    adjVal = cZoom;
                                                    break;

                                                case ElementReference::cTwist:
                                                    adjVal = cTwist;
                                                    break;
                                                
                                                default:  break;                                

                                            }
                                            break;
                                        
                                        case ElementType::array:

                                            uint8_t index = layer[i]->parameter[j]->element[k].index;

                                            switch (layer[i]->parameter[j]->element[k].ref) {

                                                case ElementReference::moveLinear:{
                                                    adjVal = move.linear[index];
                                                    }break;

                                                case ElementReference::moveRadial:{
                                                    adjVal = move.radial[index];
                                                    }break;

                                                case ElementReference::moveDirectional:{
                                                    adjVal = move.directional[index];
                                                    }break;

                                                case ElementReference::moveNoiseAngle:{
                                                    adjVal = move.noise_angle[index];
                                                    }break;
                                            }
                                            break;
                                    }
                                    
                                    if (k == 0) {
                                        // First element - just use the value (with potential negation)
                                        result = (layer[i]->parameter[j]->element[0].op == ElementOperator::SUBTRACT) ? -adjVal : adjVal;
                                    } 
                                    
                                    else {
                                        // Subsequent elements - apply operation to running result
                                        switch (layer[i]->parameter[j]->element[k].op) {
                                            case ElementOperator::ADD:
                                                result += adjVal;
                                                break;
                                            case ElementOperator::SUBTRACT:
                                                result -= adjVal;
                                                break;
                                            case ElementOperator::MULTIPLY:
                                                result *= adjVal;
                                                break;
                                            case ElementOperator::DIVIDE:
                                                if (adjVal != 0.0f) result /= adjVal;
                                                break;
                                        }
                                    } 
                            
                                } // element[k]
                                
                                // at this point, all of the elements for layer[i]->parameter[j] have been set;
                                // post the resulting value to the layer[i] vector of parameter values
                                
                                layer[i]->current[j] = result;
                        
                            } // parameter[j]

                            if (debug) {
                                for (uint8_t j = 0; j < 11; j++) {
                                    Serial.print("Parameter");
                                    Serial.print(j);
                                    Serial.print(": ");
                                    Serial.println(layer[i]->current[j]);
                                }
                            } 

                             // this is the show[i] value for the current x,y pixel in layer[i] 
                            show[i] = render_value(
                                layer[i]->current[0],   // dist
                                layer[i]->current[1],   // angle
                                layer[i]->current[2],   // scale_x
                                layer[i]->current[3],   // scale_y
                                layer[i]->current[4],   // scale_z
                                layer[i]->current[5],   // offset_x
                                layer[i]->current[6],   // offset_y
                                layer[i]->current[7],   // offset_z
                                layer[i]->current[8],   // z
                                layer[i]->current[9],   // low_limit
                                layer[i]->current[10]   // high_limit
                            );
                            
                            if (debug) {
                                Serial.print("show[");
                                Serial.print(i);
                                Serial.print("]: ");
                                Serial.println(show[i]);
                            }
                    
                        } // if layer[i]->layerActive
                    
                    } // all layers done

                    pixel.red = show[0];
                    pixel.green = show[1];
                    pixel.blue = show[1]/2 + show[0]/2;

                    pixel = rgb_sanity_check(pixel);
                    
                    setPixelColorInternal(x, y, pixel);

                } // y

            } // x

        } // createLayers(i)
        
        // *******************************************************************************************************

        void getTestInputs() {

            timings.master_speed = 0.01;
            
            timings.ratio[0] = 0.1;
            timings.ratio[1] = 0.13;
            timings.ratio[2] = 0.16;
      
            timings.offset[0] = 0;
            timings.offset[1] = 10;
            timings.offset[2] = 20;
            timings.offset[3] = 30;
            
            layer[0]->layerActive = true;
            
            // Clear and rebuild parameter 0
            layer[0]->parameter[0]->element.clear();
            layer[0]->parameter[0]->element.push_back(Element(ElementOperator::ADD, ElementType::variable, ElementReference::pixelDistance, 0, 0));
            layer[0]->parameter[0]->element.push_back(Element(ElementOperator::DIVIDE, ElementType::direct, ElementReference::null, 4.0f, 0));
          
            // Clear and rebuild parameter 1
            layer[0]->parameter[1]->element.clear();
            layer[0]->parameter[1]->element.push_back(Element(ElementOperator::ADD, ElementType::variable, ElementReference::pixelAngle, 0, 0));
            layer[0]->parameter[1]->element.push_back(Element(ElementOperator::MULTIPLY, ElementType::direct, ElementReference::null, 3.0f, 0));
            layer[0]->parameter[1]->element.push_back(Element(ElementOperator::ADD, ElementType::array, ElementReference::moveRadial, 0, 0));
            
            // Clear and set single-element parameters
            for (uint8_t j = 2; j <= 10; j++) {
                layer[0]->parameter[j]->element.clear();
            }
                        
            layer[0]->parameter[2]->element.push_back(Element(ElementOperator::ADD, ElementType::direct, ElementReference::null, 0.1f, 0));  
            layer[0]->parameter[3]->element.push_back(Element(ElementOperator::ADD, ElementType::direct, ElementReference::null, 0.1f, 0));
            layer[0]->parameter[4]->element.push_back(Element(ElementOperator::ADD, ElementType::direct, ElementReference::null, 0.1f, 0));
            layer[0]->parameter[5]->element.push_back(Element(ElementOperator::ADD, ElementType::direct, ElementReference::null, 0.0f, 0));
            layer[0]->parameter[6]->element.push_back(Element(ElementOperator::ADD, ElementType::direct, ElementReference::null, 0.0f, 0));
            layer[0]->parameter[7]->element.push_back(Element(ElementOperator::ADD, ElementType::direct, ElementReference::null, 0.0f, 0));
            layer[0]->parameter[8]->element.push_back(Element(ElementOperator::ADD, ElementType::direct, ElementReference::null, 0.0f, 0));
            layer[0]->parameter[9]->element.push_back(Element(ElementOperator::ADD, ElementType::direct, ElementReference::null, 0.0f, 0));
            layer[0]->parameter[10]->element.push_back(Element(ElementOperator::ADD, ElementType::direct, ElementReference::null, 1.0f, 0));




            // Clear and rebuild parameter 0
            layer[1]->parameter[0]->element.clear();
            layer[1]->parameter[0]->element.push_back(Element(ElementOperator::ADD, ElementType::variable, ElementReference::pixelDistance, 0, 0));
            layer[1]->parameter[0]->element.push_back(Element(ElementOperator::DIVIDE, ElementType::direct, ElementReference::null, 2.0f, 0));
          
            // Clear and rebuild parameter 1
            layer[1]->parameter[1]->element.clear();
            layer[1]->parameter[1]->element.push_back(Element(ElementOperator::SUBTRACT, ElementType::variable, ElementReference::pixelAngle, 0, 0));
            layer[1]->parameter[1]->element.push_back(Element(ElementOperator::MULTIPLY, ElementType::direct, ElementReference::null, 6.0f, 0));
            layer[1]->parameter[1]->element.push_back(Element(ElementOperator::ADD, ElementType::array, ElementReference::moveRadial, 2, 0));
            
            // Clear and set single-element parameters
            for (uint8_t j = 2; j <= 10; j++) {
                layer[1]->parameter[j]->element.clear();
            }
                        
            layer[1]->parameter[2]->element.push_back(Element(ElementOperator::ADD, ElementType::direct, ElementReference::null, 0.2f, 0));  
            layer[1]->parameter[3]->element.push_back(Element(ElementOperator::ADD, ElementType::direct, ElementReference::null, 0.2f, 0));
            layer[1]->parameter[4]->element.push_back(Element(ElementOperator::ADD, ElementType::direct, ElementReference::null, 0.2f, 0));
            layer[1]->parameter[5]->element.push_back(Element(ElementOperator::ADD, ElementType::direct, ElementReference::null, 0.0f, 0));
            layer[1]->parameter[6]->element.push_back(Element(ElementOperator::ADD, ElementType::direct, ElementReference::null, 0.0f, 0));
            layer[1]->parameter[7]->element.push_back(Element(ElementOperator::ADD, ElementType::direct, ElementReference::null, 0.0f, 0));
            layer[1]->parameter[8]->element.push_back(Element(ElementOperator::ADD, ElementType::direct, ElementReference::null, 0.0f, 0));
            layer[1]->parameter[9]->element.push_back(Element(ElementOperator::ADD, ElementType::direct, ElementReference::null, 0.0f, 0));
            layer[1]->parameter[10]->element.push_back(Element(ElementOperator::ADD, ElementType::direct, ElementReference::null, 1.0f, 0));


            /*if(debug){
                Serial.print("timings.master_speed: ") ; Serial.println(timings.master_speed);
                Serial.print("timings.ratio[0]: ") ; Serial.println(timings.ratio[0]);
                Serial.print("timings.ratio[1]: ") ; Serial.println(timings.ratio[1]);
                Serial.print("timings.ratio[2]: ") ; Serial.println(timings.ratio[2]);
                Serial.print("timings.offset[0]: ") ; Serial.println(timings.offset[0]);
                Serial.print("timings.offset[1]: ") ; Serial.println(timings.offset[1]);
                Serial.print("timings.offset[2]: ") ; Serial.println(timings.offset[2]);
                Serial.print("timings.offset[3]: ") ; Serial.println(timings.offset[3]);
            }*/
        }

        //********************************************************************************************************************
        // PATTERN GENERATOR *************************************************************************************************

        // each pass through main loop, show the Frame
        void Pattern() { 

            if (debug) {Serial.println("Pattern start");} 

            getTestInputs();

            calculate_modulators(timings);
            
            createLayers();
          
            if (debug) {Serial.println("Pattern end");}     
        
        } // Pattern()

        //*******************************************************************************

    }; // class ANIMartRIX

}; // namespace animartrix_detail

// End fast math optimizations
#if FL_ANIMARTRIX_USES_FAST_MATH
FL_OPTIMIZATION_LEVEL_O3_END
FL_FAST_MATH_END
#endif