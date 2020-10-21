/*
  *******************************************************************************
 * @file 4WD encoder value
 * @brief calculate rotation velocity
 * @author Ramune6110
 * @date 2020 10/09
 * SYSTEM            | Arduino MEGA 2560
  *******************************************************************************
*/
/***********************************************************************
 * Header files 
 **********************************************************************/
#define front_right_outputA 38
#define front_right_outputB 39
#define front_left_outputA  36
#define front_left_outputB  37
#define back_right_outputA  34
#define back_right_outputB  35
#define back_left_outputA   32
#define back_left_outputB   33

//#define SHOW_ENCODER
#define SHOW_ROTATION_VELOCITY

/***********************************************************************
 * Global variables
 **********************************************************************/
int counter[4] = {0}; 
int current_A[4] = {0};
int preoutput__A[4] = {0};  

// ロータリーエンコーダの1回転あたりのパルス数
int rotary_encoder_resolution = 80;

float rotation_angle[4] = {0.0f};
float pre_rotation_angle[4] = {0.0f};
float rotation_angular_velocity[4] = {0.0f};

float dt = 0.0f;
float pre_time = 0.0f;
float current_time = 0.0f;

/**
***********************************************************************
* Setup
***********************************************************************
*/
void setup() { 
    // wheel front left
    pinMode (front_left_outputA,INPUT);
    pinMode (front_left_outputB,INPUT);
    // wheel front right
    pinMode (front_right_outputA,INPUT);
    pinMode (front_right_outputB,INPUT);
    // wheel back left
    pinMode (back_left_outputA,INPUT);
    pinMode (back_left_outputB,INPUT);
    // wheel back right
    pinMode (back_right_outputA,INPUT);
    pinMode (back_right_outputB,INPUT);

    Serial.begin (9600);
   
    // wheel back left
    preoutput__A[0] = digitalRead(back_left_outputA);   
    // wheel back right
    preoutput__A[1] = digitalRead(back_right_outputA);
    // wheel right left
    preoutput__A[2] = digitalRead(front_left_outputA);
    // wheel right right
    preoutput__A[3] = digitalRead(front_right_outputA); 
} 

/**
***********************************************************************
* MAIN LOOP
***********************************************************************
*/
void loop() { 
    /**
     ***********************************************************************
     * Encoder
     ***********************************************************************
    */

    current_time = millis();

    // current value
    // wheel back left
    current_A[0] = digitalRead(back_left_outputA); 
    // wheel back right
    current_A[1] = digitalRead(back_right_outputA); 
    // wheel right left
    current_A[2] = digitalRead(front_left_outputA); 
    // wheel right right
    current_A[3] = digitalRead(front_right_outputA); 

    // wheel back left
    if (current_A[0] != preoutput__A[0]){     
        
        if (digitalRead(back_left_outputB) != current_A[0]) { 
          counter[0] ++;
        } else {
          counter[0] --;
        }

        #ifdef SHOW_ENCODER
            Serial.print("Position back left: ");
            Serial.println(counter[0]);
        #endif
    } 
    
    // wheel back right
    if (current_A[1] != preoutput__A[1]){     
        
        if (digitalRead(back_right_outputB) != current_A[1]) { 
          counter[1] ++;
        } else {
          counter[1] --;
        }

        #ifdef SHOW_ENCODER
            Serial.print("Position back right: ");
            Serial.println(counter[1]);
        #endif
    } 

    // wheel front left
    if (current_A[2] != preoutput__A[2]){     
        
        if (digitalRead(front_left_outputB) != current_A[2]) { 
          counter[2] ++;
        } else {
          counter[2] --;
        }

        #ifdef SHOW_ENCODER
            Serial.print("Position front left: ");
            Serial.println(counter[2]); 
        #endif
    } 

    // wheel front right
    if (current_A[3] != preoutput__A[3]){     
        
        if (digitalRead(front_right_outputB) != current_A[3]) { 
          counter[3] ++;
        } else {
          counter[3] --;
        }

        #ifdef SHOW_ENCODER
            Serial.print("Position front right: ");
            Serial.println(counter[3]);
        #endif
    } 

    /**
     ***********************************************************************
     * rotation velocity
     ***********************************************************************
    */

    // sampling time 0.01s
    dt = (current_time - pre_time) / 1000;
    //Serial.print("dt: ");
    //Serial.println(dt);
    
    //-------------------------------------------
    // rotation angle
    //-------------------------------------------
    rotation_angle[0] = counter[0] * (2 * 3.14f)/(4 * rotary_encoder_resolution);
    rotation_angle[1] = counter[1] * (2 * 3.14f)/(4 * rotary_encoder_resolution);
    rotation_angle[2] = counter[2] * (2 * 3.14f)/(4 * rotary_encoder_resolution);
    rotation_angle[3] = counter[3] * (2 * 3.14f)/(4 * rotary_encoder_resolution);

    //-------------------------------------------
    // rotation velocity
    //-------------------------------------------
    rotation_angular_velocity[0] = (rotation_angle[0] - pre_rotation_angle[0]) / dt;
    rotation_angular_velocity[1] = (rotation_angle[1] - pre_rotation_angle[1]) / dt;
    rotation_angular_velocity[2] = (rotation_angle[2] - pre_rotation_angle[2]) / dt;
    rotation_angular_velocity[3] = (rotation_angle[3] - pre_rotation_angle[3]) / dt;

    #ifdef SHOW_ROTATION_VELOCITY
        Serial.print("Velocity back left: ");
        Serial.println(rotation_angular_velocity[0]);
        Serial.print("Velocity back right: ");
        Serial.println(rotation_angular_velocity[1]);
        Serial.print("Velocity front left: ");
        Serial.println(rotation_angular_velocity[2]);
        Serial.print("Velocity front right: ");
        Serial.println(rotation_angular_velocity[3]);
    #endif 
    
    // preserve
     // wheel back left
    preoutput__A[0] = current_A[0]; 
    pre_rotation_angle[0] = rotation_angle[0];
    // wheel back right
    preoutput__A[1] = current_A[1];
    pre_rotation_angle[1] = rotation_angle[1]; 
    // wheel right left
    preoutput__A[2] = current_A[2]; 
    pre_rotation_angle[2] = rotation_angle[2]; 
    // wheel right right
    preoutput__A[3] = current_A[3];
    pre_rotation_angle[3] = rotation_angle[3]; 

    pre_time = current_time;
}