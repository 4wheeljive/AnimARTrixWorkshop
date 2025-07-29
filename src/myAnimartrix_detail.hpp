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

#define num_oscillators 10

namespace animartrix_detail {

FASTLED_USING_NAMESPACE

    struct render_parameters {
        float center_x = (999 / 2) - 0.5; // center of the matrix
        float center_y = (999 / 2) - 0.5;
        float dist, angle;
        float scale_x = .1; // smaller values = zoom in
        float scale_y = .1;
        float scale_z = .1;
        float offset_x, offset_y, offset_z;
        float z;
        float low_limit = 0; // getting contrast by raising the black point
        float high_limit = 1;
    };

    struct oscillators {
        float master_speed; // global transition speed
        float offset[num_oscillators];  // oscillators can be shifted by a time offset
        float ratio[num_oscillators]; // speed ratios for the individual oscillators
    };

    struct modulators {
        float linear[num_oscillators];      // returns 0 to FLT_MAX
        float radial[num_oscillators];      // returns 0 to 2*PI
        float directional[num_oscillators]; // returns -1 to 1
        float noise_angle[num_oscillators]; // returns 0 to 2*PI
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
        int num_x; // matrix width
        int num_y; // matrix height

        float speed_factor = 1; // 0.1 to 10

        float radial_filter_radius = 23.0; // on 32x32, use 11 for 16x16
        float radialDimmer = 1;
        float radialFilterFalloff = 1;

        bool serpentine;

        render_parameters animation;    // all animation parameters in one place (an instance of the render_parameters structure)
        oscillators timings;            // all speed settings in one place (an instance of the oscillators structure)
        modulators move;                // all oscillator-based movers and shifters in one place (an instance of the modulators structure)
        rgb pixel;                      // an instance of the rgb structure

        fl::HeapVector<fl::HeapVector<float>>
            polar_theta; // look-up table for polar angles
        fl::HeapVector<fl::HeapVector<float>>
            distance; // look-up table for polar distances

        unsigned long a, b, c; // for time measurements

        //float show1, show2, show3;
        float show[10] = {0,1,2,3,4,5,6,7,8,9};

        ANIMartRIX() {}

        ANIMartRIX(int w, int h) { this->init(w, h); }

        virtual ~ANIMartRIX() {}

        virtual uint16_t xyMap(uint16_t x, uint16_t y) = 0;

        uint32_t currentTime = 0;
        void setTime(uint32_t t) { currentTime = t; }
        uint32_t getTime() { return currentTime ? currentTime : millis(); }

        void init(int w, int h) {
            animation = render_parameters();
            timings = oscillators();
            move = modulators();
            pixel = rgb();

            this->num_x = w;
            this->num_y = h;

            this->radial_filter_radius = std::min(w,h) * 0.65;
        
            // precalculate all polar coordinates; polar origin is set to matrix centre
            render_polar_lookup_table(
                (num_x / 2) - 0.5,
                (num_y / 2) - 0.5);  
            
            // Set default speed ratio for the oscillators. Not all effects set their own.
            timings.master_speed = 0.01;
        }

        void setSpeedFactor(float speed) { this->speed_factor = speed; }

        float radialFilterFactor( float radius, float distance, float falloff) {
            if (distance >= radius) return 0.0f;
            float factor = 1.0f - (distance / radius);
            return powf(factor, falloff);
        }

        // Dynamic darkening methods *************************************

        float subtract(float &a, float &b) { return a - b; }

        float multiply(float &a, float &b) { return a * b / 255.f; }

        // makes low brightness darker
        // sets the black point high = more contrast
        // animation.low_limit should be 0 for best results
        float colorburn(float &a, float &b) {
            return (1 - ((1 - a / 255.f) / (b / 255.f))) * 255.f;
        }

        // Dynamic brightening methods **********************************

        float add(float &a, float &b) { return a + b; }

        // makes bright even brighter
        // reduces contrast
        float screen(float &a, float &b) {
            return (1 - (1 - a / 255.f) * (1 - b / 255.f)) * 255.f;
        }

        float colordodge(float &a, float &b) { return (a / (255.f - b)) * 255.f; }

        //***************************************************************
        
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

        void calculate_oscillators(oscillators &timings) {

            // global animation speed
            double runtime = getTime() * timings.master_speed * speed_factor; 

            for (int i = 0; i < num_oscillators; i++) {

                // continously rising offsets, returns 0 to max_float
                move.linear[i] = 
                    (runtime + timings.offset[i]) * timings.ratio[i];

                // angle offsets for continous rotation, returns 0 to 2 * PI
                move.radial[i] = 
                    fmodf(move.linear[i], 2 * PI); 

                // directional offsets or factors, returns -1 to 1
                move.directional[i] = 
                    FL_SIN_F(move.radial[i]); 

                // noise based angle offset, returns 0 to 2 * PI
                move.noise_angle[i] =
                    PI * (1 + pnoise(move.linear[i], 0, 0));
            
            }
        }
    
        //***************************************************************
        void run_default_oscillators(float master_speed ) {
                timings.master_speed = master_speed;

                // speed ratios for the oscillators, higher values = faster transitions
                timings.ratio[0] = 1; 
                timings.ratio[1] = 2;
                timings.ratio[2] = 3;
                timings.ratio[3] = 4;
                timings.ratio[4] = 5;
                timings.ratio[5] = 6;
                timings.ratio[6] = 7;
                timings.ratio[7] = 8;
                timings.ratio[8] = 9;
                timings.ratio[9] = 10;

                timings.offset[0] = 000;
                timings.offset[1] = 100;
                timings.offset[2] = 200;
                timings.offset[3] = 300;
                timings.offset[4] = 400;
                timings.offset[5] = 500;
                timings.offset[6] = 600;
                timings.offset[7] = 700;
                timings.offset[8] = 800;
                timings.offset[9] = 900;

                calculate_oscillators(timings);
            }

        //***************************************************************

        // Convert the 2 polar coordinates back to cartesian ones & also apply all
        // 3d transitions. Calculate the noise value at this point based on the 5
        // dimensional manipulation of the underlaying coordinates.

    
        // an instance of the render_parameters structure called animation passes a layer's collection of parameters through
        // the render_value function    

        // returns a float between 0 and 255 that will be translated into the r, g and/or b value of a single pixel, 
        // depending on how colors are mapped 
        
        // EXAMPLE:  float show1 = render_value(animation)

        // Ideally this should only be used for Layers that have a rotational component. Otherwise, there is no need
        // for any polar/cartesian conversions

        // An initial approach for me could be to have each Layer instance result in xxx
        // What my Layers should do is map my "parameter factory" values to the animation/render_parameters structure           


        float render_value(render_parameters &animation) {

            // convert polar coordinates back to cartesian ones

            float newx = (animation.offset_x + animation.center_x -
                        (FL_COS_F(animation.angle) * animation.dist)) *
                        animation.scale_x;
            float newy = (animation.offset_y + animation.center_y -
                        (FL_SIN_F(animation.angle) * animation.dist)) *
                        animation.scale_y;
            float newz = (animation.offset_z + animation.z) * animation.scale_z;

            // render noisevalue at this new cartesian point

            float raw_noise_field_value = pnoise(newx, newy, newz);

            if (raw_noise_field_value < animation.low_limit)
                raw_noise_field_value = animation.low_limit;
            if (raw_noise_field_value > animation.high_limit)
                raw_noise_field_value = animation.high_limit;

            float scaled_noise_value =
                map_float(raw_noise_field_value, animation.low_limit,
                        animation.high_limit, 0, 255);

            return scaled_noise_value;
        }

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
        }   //*******************************************&&&&&&&&&&&&&&&&&&&&&&&&& */

        virtual void setPixelColorInternal(int x, int y, rgb pixel) = 0;



        //********************************************************************************************************************
        // LAYERS ************************************************************************************************************

        //**************************************************************************************** 
        // ADJUSTMENT STRUCTURE 

        // Enumeration for adjustment operators
        enum class AdjustmentOperator {
            ADD,      // +
            SUBTRACT, // -
            MULTIPLY  // *
        };

        // Core variant type - can be a float value, pointer to float, pointer to float array, or lambda function
        using CoreVariant = std::variant<float, float*, float**, std::function<float()>>;
                
        // Adjustment structure with core, optional index, and operator
        struct Adjustment {
            CoreVariant core;
            uint8_t index = 0;           // Optional index for array variables
            AdjustmentOperator op = AdjustmentOperator::ADD;  // Default to addition
            
            // Constructor with float core
            Adjustment(float value, AdjustmentOperator operation = AdjustmentOperator::ADD)
                : core(value), op(operation) {}
            
            // Constructor with pointer to float variable
            Adjustment(float* variable, AdjustmentOperator operation = AdjustmentOperator::ADD)
                : core(variable), op(operation) {}
            
            // Constructor with pointer to float array and index
            Adjustment(float* array, uint8_t arrayIndex, AdjustmentOperator operation = AdjustmentOperator::ADD)
                : core(array), index(arrayIndex), op(operation) {}
            
            // Constructor with lambda function
            Adjustment(std::function<float()> func, AdjustmentOperator operation = AdjustmentOperator::ADD)
                : core(func), op(operation) {}

           
           // Get the core value as float
            float getCoreValue() const {
                return std::visit([this](const auto& value) -> float {
                    using T = std::decay_t<decltype(value)>;
                    
                    if constexpr (std::is_same_v<T, float>) {
                        // Direct float value
                        return value;
                    }
                    else if constexpr (std::is_same_v<T, float*>) {
                        // Pointer to float or float array
                        if (value != nullptr) {
                            return value[index]; // Works for both single variable (index=0) and arrays
                        }
                        return 0.0f;
                    }
                    else if constexpr (std::is_same_v<T, float**>) {
                        // Pointer to pointer (for more complex scenarios)
                        if (value != nullptr && *value != nullptr) {
                            return (*value)[index];
                        }
                        return 0.0f;
                    }
                    else if constexpr (std::is_same_v<T, std::function<float()>>) {
                        // Execute the lambda function
                        if (value) {
                            return value();
                        }
                        return 0.0f;
                    }
                    else {
                        return 0.0f; // Fallback
                    }
                }, core);
            }

        };// struct Adjustment

                                        
        //**************************************************************************************** 
        // PARAMETER CLASS - each parameter returns a float
       
        // Base variant type for runtime flexibility
        using BaseVariant = std::variant<float, float*, float**, std::function<float()>>;
        
        class Parameter {

          public:
            BaseVariant base;
            std::vector<Adjustment> adjustment;

            // Default constructor - base defaults to 0.0f
            Parameter() : base(0.0f) {}
            
            // Constructor with float base value
            explicit Parameter(float baseValue) : base(baseValue) {}
            
            // Constructor with pointer to float base
            explicit Parameter(float* basePtr) : base(basePtr) {}
            
            // New constructor for lambda functions
            explicit Parameter(std::function<float()> func) : base(func) {}

            // Constructor with base and adjustments
            Parameter(float baseValue, const std::vector<Adjustment>& adjs)
                : base(baseValue), adjustment(adjs) {}
            
            Parameter(float* basePtr, const std::vector<Adjustment>& adjs)
                : base(basePtr), adjustment(adjs) {}
            
            // Get base value as float
            float getBaseValue() const {
                return std::visit([](const auto& value) -> float {
                    using T = std::decay_t<decltype(value)>;
                    
                    if constexpr (std::is_same_v<T, float>) {
                        return value;
                    }
                    else if constexpr (std::is_same_v<T, float*>) {
                        if (value != nullptr) {
                            return *value;
                        }
                        return 0.0f;
                    }
                    else if constexpr (std::is_same_v<T, float**>) {
                        if (value != nullptr && *value != nullptr) {
                            return **value;
                        }
                        return 0.0f;
                    }
                    else if constexpr (std::is_same_v<T, std::function<float()>>) {
                        // Execute the lambda function
                        if (value) {
                            return value();
                        }
                        return 0.0f;
                    }
                    else {
                        return 0.0f; // Fallback
                    }
                }, base);
            }
            
            // Main function - returns the computed float value
            float getValue() const {
            
                // Start with base value
                float result = getBaseValue();
                if (debug) {
                    Serial.print("    getValue() base: "); Serial.println(result);
                }
                
                // Apply adjustments sequentially
                for (size_t i = 0; i < adjustment.size(); i++) {
                    float adjValue = adjustment[i].getCoreValue();
                    float oldResult = result;
                    
                    if (debug) {
                        Serial.print("    Adjustment["); Serial.print(i); Serial.print("] value: "); Serial.println(adjValue);
                    }

                    switch (adjustment[i].op) {
                        case AdjustmentOperator::ADD:
                            result += adjValue;
                            if (debug) {
                                Serial.print("    "); Serial.print(oldResult); Serial.print(" + "); Serial.print(adjValue); 
                                Serial.print(" = "); Serial.println(result);
                            }
                            break;
                        case AdjustmentOperator::SUBTRACT:
                            result -= adjValue;
                            if (debug) {
                                Serial.print("    "); Serial.print(oldResult); Serial.print(" + "); Serial.print(adjValue); 
                                Serial.print(" = "); Serial.println(result);
                            }
                            break;
                        case AdjustmentOperator::MULTIPLY:
                            result *= adjValue;
                            if (debug) {
                                Serial.print("    "); Serial.print(oldResult); Serial.print(" + "); Serial.print(adjValue); 
                                Serial.print(" = "); Serial.println(result);
                            }
                            break;
                    }
                }
                
                if (debug) {
                    Serial.print("    getValue() final: "); Serial.println(result);
                }
                
                return result;
            }

            // Utility methods
            void setBase(float newBase) { base = newBase; }
            void setBase(float* newBasePtr) { base = newBasePtr; }
            void setBase(std::function<float()> func) { base = func; }
            
            // Type checking methods
            bool isBaseDirectValue() const {
                return std::holds_alternative<float>(base);
            }
            
            bool isBasePointer() const {
                return std::holds_alternative<float*>(base);
            }
            
            bool isBaseFunction() const {
                return std::holds_alternative<std::function<float()>>(base);
            }
            
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
            
            // Get parameter count (always 11)
            // change to count number of Parameters !=0 

            static constexpr size_t getParameterCount() {
                return 11;
            }
            
            // Get all parameter values as floats
            void getParameterValues(float values[1]) const {
                for (size_t i = 0; i < 11; ++i) {
                    values[i] = parameter[i]->getValue();
                }
            }
            
            // Get specific parameter value by index
            float getParameterValue(size_t index) const {
                if (index < 11) {
                    return parameter[index]->getValue();
                }
                return 0.0f;
            }
        }; // Layer class

        //************************************************************/

        Layer layer1;
        Layer layer2;
        Layer layer3;
        Layer layer4;
        Layer layer5;
                
        Layer* layer[5] = {&layer1, &layer2, &layer3, &layer4, &layer5};



        float modFactorDistance(uint8_t i) {

            if () {     // if there are any adjustments to default base,
                        // then process them as applicable

                // Look to see if there are any active adjustments

                // if there are active static adjustments...


                // if there are active dynamic adjustments...



                // use getValue or another method to process
                   
                // layer[i]->distance_xy.getValue();


            // return result     
            }

            else {return 1.0f;}



        }

        float modFactorAngle(uint8_t i) {

            if () {     // if there are any adjustments to default base,
                        // then process them as applicable

                // Look to see if there are any active adjustments

                // if there are active static adjustments...


                // if there are active dynamic adjustments...



                // use getValue or another method to process
                   
                // layer[i]->angle.getValue();


            // return result     
            }

            else {return 1.0f;}



        }

/*
                            animation.scale_x = layer[i]->scale_x.getValue();
                        
                            animation.scale_y = layer[i]->scale_y.getValue();
                            animation.scale_z = layer[i]->scale_z.getValue();
                            animation.offset_x = layer[i]->offset_x.getValue();
                            animation.offset_y = layer[i]->offset_y.getValue();
                            animation.offset_z = layer[i]->offset_z.getValue();
                            animation.z = layer[i]->z.getValue();
                            animation.low_limit = layer[i]->low_limit.getValue();
                            animation.high_limit = layer[i]->high_limit.getValue();

*/

        
        float currentScale_x(uint8_t i) {

            return 0.0f;
        
        };
        float currentScale_y(uint8_t i) {
            return 0.0f;
        };
        float currentScale_y(uint8_t i) {
            return 0.0f;
        };
        float currentOffset_x(uint8_t i) {
           // implement the method here
           // if it's a constant, return it

           // if it's based on an oscillator/modulator,
           // then return that value directly 


            return 0.0f;
        };
        float currentOffset_y(uint8_t i) {
            return 0.0f;
        };
        float currentOffset_z(uint8_t i) {
            return 0.0f;
        };
        float currentZ(uint8_t i) {
            return 0.0f;
        };
        
        float currentLow_Limit(uint8_t i) { return 0.0f; };
        float currentHigh_Limit(uint8_t i) { return 1.0f; };



        // *************** USE BLE HANDLERS TO PUSH CHANGES ONLY WHEN NEEDED

        /*
            //EXAMPLE state of UI input:
            layer1.layerActive = true;
            layer1.distance_xy.setBase([&]() { return distance[x][y]; });
            layer1.distance_xy.adjustment.push_back(Adjustment(.25f, AdjustmentOperator::MULTIPLY));
           
            layer1.angle.setBase([&]() { return polar_theta[x][y]; });
            layer1.angle.adjustment.push_back(Adjustment(3.0f, AdjustmentOperator::MULTIPLY));
            layer1.angle.adjustment.push_back(Adjustment(move.radial, 0, AdjustmentOperator::ADD));
            layer1.angle.adjustment.push_back(Adjustment([&]() { return distance[x][y]; }, AdjustmentOperator::SUBTRACT));
           
            layer1.scale_x.base = .1f;  
            layer1.scale_y.base = .1f;
            layer1.scale_z.base = .1f;
            layer1.offset_x.base = &move.linear[0];
            layer1.high_limit.base = 1.0f;
        */


        // BLE - Set variable values directly? Or send instruction messages?

            // special/dedicated characteristic?


            // can moderate the adjustment count only when needed 


            // all dynamic adjustments point directly to the underlying timer; they don't pass values from one place to another


    
        //********************************************************************************************************************
        // EFFECTS ***********************************************************************************************************

        void Test() {
            Serial.println("Test started");
            
            int x = 0, y = 0;

            timings.master_speed = 0.01;
            timings.ratio[0] = 0.1;
            timings.ratio[1] = 0.13;
            timings.ratio[2] = 0.16;
      
            timings.offset[1] = 10;
            timings.offset[2] = 20;
            timings.offset[3] = 30;

            calculate_oscillators(timings);



            //draw a frame
            for (x = 0; x < num_x; x++) {
                for (y = 0; y < num_y; y++) {
                    
                    // draw layers 
                    for (uint8_t i = 0; i < 5 ; i++) {
                
                        if(layer[i]->layerActive) {
                        
                            float currentDistance = distance[x][y];
                            float currentAngle = polar_theta[x][y];
                            
                            // need to adjust to allow for + as alternate operator
                            animation.dist = currentDistance * modFactorDistance(i) ; 
                            animation.angle = currentAngle * modFactorAngle(i);
						    animation.scale_x = currentScale_x(i);
							animation.scale_y = currentScale_y(i);
                            animation.scale_z = currentScale_y(i);
                            animation.offset_x = currentOffset_x(i);
                            animation.offset_y = currentOffset_y(i);
                            animation.offset_z = currentOffset_z(i);
                            animation.z = currentZ(i);
                            animation.low_limit =currentLow_Limit(i);
                            animation.high_limit = currentHigh_Limit(i);
                            
                            show[i] = render_value(animation);
                            //Serial.print("Show: ");
                            //Serial.println(show[i]);

                        }
                       
                    }
                        
                    /*
					animation.dist = distance[x][y] / 4 ;
					animation.angle =
						3 * polar_theta[x][y] 
						+ move.radial[0] 
						- distance[x][y]; 
					animation.scale_z = .1;
					animation.scale_y = .1 ;
					animation.scale_x = .1 ;
					animation.offset_x = move.linear[0];
					animation.offset_y = 0;
					animation.offset_z = 0;
					animation.z = 0;
					
                    show[0] = render_value(animation) ;

                    */

                    pixel.red = show[0];
                    pixel.green = show[0]/2;
                    pixel.blue = show[0]/3;

                    pixel = rgb_sanity_check(pixel);
                    setPixelColorInternal(x, y, pixel);

                }
            }
            
            Serial.println("Test completed");
        } // Test()

        //*******************************************************************************

    }; // class ANIMartRIX

}; // namespace animartrix_detail

// End fast math optimizations
#if FL_ANIMARTRIX_USES_FAST_MATH
FL_OPTIMIZATION_LEVEL_O3_END
FL_FAST_MATH_END
#endif