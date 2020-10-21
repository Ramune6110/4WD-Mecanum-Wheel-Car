/*
  *******************************************************************************
 * @file 4WD encoder value
 * @brief calculate rotation velocity
 * @author Ramune6110
 * @date 2020 10/10
 * SYSTEM            | Arduino MEGA 2560
  *******************************************************************************
*/
/***********************************************************************
 * Header files 
 **********************************************************************/
#include <MsTimer2.h>

/***********************************************************************
 * Preprocessor
 **********************************************************************/
#define SPEED               140    
#define TURN_SPEED          160    
#define speedPinR           9   //  Front Wheel PWM pin connect Right MODEL-X ENA 
#define RightMotorDirPin1   22    //Front Right Motor direction pin 1 to Right MODEL-X IN1  (K1)
#define RightMotorDirPin2   24   //Front Right Motor direction pin 2 to Right MODEL-X IN2   (K1)                                 
#define LeftMotorDirPin1    26    //Front Left Motor direction pin 1 to Right MODEL-X IN3 (K3)
#define LeftMotorDirPin2    28   //Front Left Motor direction pin 2 to Right MODEL-X IN4 (K3)
#define speedPinL           10   //  Front Wheel PWM pin connect Right MODEL-X ENB

#define speedPinRB          11   //  Rear Wheel PWM pin connect Left MODEL-X ENA 
#define RightMotorDirPin1B  5    //Rear Right Motor direction pin 1 to Left  MODEL-X IN1 ( K1)
#define RightMotorDirPin2B  6    //Rear Right Motor direction pin 2 to Left  MODEL-X IN2 ( K1) 
#define LeftMotorDirPin1B   7    //Rear Left Motor direction pin 1 to Left  MODEL-X IN3  (K3)
#define LeftMotorDirPin2B   8  //Rear Left Motor direction pin 2 to Left  MODEL-X IN4 (K3)
#define speedPinLB          12    //  Rear Wheel PWM pin connect Left MODEL-X ENB

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

/***********************************************************************
 * Prototype declaration
 **********************************************************************/
void go_advance(int speed);
void go_back(int speed);

void right_shift(int speed_fl_fwd,int speed_rl_bck ,int speed_rr_fwd,int speed_fr_bck);
void left_shift(int speed_fl_bck,int speed_rl_fwd ,int speed_rr_bck,int speed_fr_fwd);

void left_turn(int speed);
void right_turn(int speed);

void left_back(int speed);
void right_back(int speed);

void clockwise(int speed);
void countclockwise(int speed);

void FR_fwd(int speed);
void FR_bck(int speed);
void FL_fwd(int speed);
void FL_bck(int speed);
void RR_fwd(int speed);
void RR_bck(int speed);
void RL_fwd(int speed);
void RL_bck(int speed);

void stop_Stop();

void init_GPIO();

void update_encoder();

/**
***********************************************************************
* Setup
***********************************************************************
*/
void setup() { 

    Serial.begin (9600);

    // moter pin
    init_GPIO();
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

    // wheel back left
    preoutput__A[0] = digitalRead(back_left_outputA);   
    // wheel back right
    preoutput__A[1] = digitalRead(back_right_outputA);
    // wheel right left
    preoutput__A[2] = digitalRead(front_left_outputA);
    // wheel right right
    preoutput__A[3] = digitalRead(front_right_outputA); 

    // Interruption for rotary encoder reading
    //MsTimer2::set(100000000, update_encoder);       
    //MsTimer2::start();
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

    /*current_time = millis();

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
    }*/

    /**
     ***********************************************************************
     * rotation velocity
     ***********************************************************************
    */

    /*dt = (current_time - pre_time) / 1000;
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

    pre_time = current_time;*/

    sequence_control();
}

/**
***********************************************************************
* rotation velocity
***********************************************************************
*/
void update_encoder()
{
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

/**
***********************************************************************
* moter control
***********************************************************************
*/

//Pins initialize
void init_GPIO()
{
  pinMode(RightMotorDirPin1, OUTPUT); 
  pinMode(RightMotorDirPin2, OUTPUT); 
  pinMode(speedPinL, OUTPUT);  
 
  pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT); 
  pinMode(speedPinR, OUTPUT);
  pinMode(RightMotorDirPin1B, OUTPUT); 
  pinMode(RightMotorDirPin2B, OUTPUT); 
  pinMode(speedPinLB, OUTPUT);  
 
  pinMode(LeftMotorDirPin1B, OUTPUT);
  pinMode(LeftMotorDirPin2B, OUTPUT); 
  pinMode(speedPinRB, OUTPUT);
   
  stop_Stop();
}

void sequence_control()
{
    go_advance(SPEED);
    delay(1000);
    stop_Stop();
    delay(1000);

    go_back(SPEED);
    delay(1000);
    stop_Stop();
    delay(1000);

    left_turn(TURN_SPEED);
    delay(1000);
    stop_Stop();
    delay(1000);

    right_turn(TURN_SPEED);
    delay(1000);
    stop_Stop();
    delay(1000);

    right_shift(200,200,200,200); //right shift
    delay(1000);
    stop_Stop();
    delay(1000);

    left_shift(200,200,200,200); //left shift
    delay(1000);
    stop_Stop();
    delay(1000);

    left_shift(200,0,200,0); //left diagonal back
    delay(1000);
    stop_Stop();
    delay(1000);

    right_shift(200,0,200,0); //right diagonal ahead
    delay(1000);
    stop_Stop();
    delay(1000);

    left_shift(0,200,0,200); //left diagonal ahead
    delay(1000);
    stop_Stop();
    delay(1000);

    right_shift(0,200,0,200); //right diagonal back
    delay(1000);
    stop_Stop();
    delay(1000);
}

void go_advance(int speed){
   RL_fwd(speed);
   RR_fwd(speed);
   FR_fwd(speed);
   FL_fwd(speed); 
}
void go_back(int speed){
   RL_bck(speed);
   RR_bck(speed);
   FR_bck(speed);
   FL_bck(speed); 
}
void right_shift(int speed_fl_fwd,int speed_rl_bck ,int speed_rr_fwd,int speed_fr_bck) {
  FL_fwd(speed_fl_fwd); 
  RL_bck(speed_rl_bck); 
  RR_fwd(speed_rr_fwd);
  FR_bck(speed_fr_bck);
}
void left_shift(int speed_fl_bck,int speed_rl_fwd ,int speed_rr_bck,int speed_fr_fwd){
   FL_bck(speed_fl_bck);
   RL_fwd(speed_rl_fwd);
   RR_bck(speed_rr_bck);
   FR_fwd(speed_fr_fwd);
}

void left_turn(int speed){
   RL_bck(0);
   RR_fwd(speed);
   FR_fwd(speed);
   FL_bck(0); 
}
void right_turn(int speed){
   RL_fwd(speed);
   RR_bck(0);
   FR_bck(0);
   FL_fwd(speed); 
}
void left_back(int speed){
   RL_fwd(0);
   RR_bck(speed);
   FR_bck(speed);
   FL_fwd(0); 
}
void right_back(int speed){
   RL_bck(speed);
   RR_fwd(0);
   FR_fwd(0);
   FL_bck(speed); 
}
void clockwise(int speed){
   RL_fwd(speed);
   RR_bck(speed);
   FR_bck(speed);
   FL_fwd(speed); 
}
void countclockwise(int speed){
   RL_bck(speed);
   RR_fwd(speed);
   FR_fwd(speed);
   FL_bck(speed); 
}

void FR_fwd(int speed)  //front-right wheel forward turn
{
  digitalWrite(RightMotorDirPin1,HIGH);
  digitalWrite(RightMotorDirPin2,LOW); 
  analogWrite(speedPinR,speed);
}
void FR_bck(int speed) // front-right wheel backward turn
{
  digitalWrite(RightMotorDirPin1,LOW);
  digitalWrite(RightMotorDirPin2,HIGH); 
  analogWrite(speedPinR,speed);
}
void FL_fwd(int speed) // front-left wheel forward turn
{
  digitalWrite(LeftMotorDirPin1,HIGH);
  digitalWrite(LeftMotorDirPin2,LOW);
  analogWrite(speedPinL,speed);
}
void FL_bck(int speed) // front-left wheel backward turn
{
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,HIGH);
  analogWrite(speedPinL,speed);
}

void RR_fwd(int speed)  //rear-right wheel forward turn
{
  digitalWrite(RightMotorDirPin1B, HIGH);
  digitalWrite(RightMotorDirPin2B,LOW); 
  analogWrite(speedPinRB,speed);
}
void RR_bck(int speed)  //rear-right wheel backward turn
{
  digitalWrite(RightMotorDirPin1B, LOW);
  digitalWrite(RightMotorDirPin2B,HIGH); 
  analogWrite(speedPinRB,speed);
}
void RL_fwd(int speed)  //rear-left wheel forward turn
{
  digitalWrite(LeftMotorDirPin1B,HIGH);
  digitalWrite(LeftMotorDirPin2B,LOW);
  analogWrite(speedPinLB,speed);
}
void RL_bck(int speed)    //rear-left wheel backward turn
{
  digitalWrite(LeftMotorDirPin1B,LOW);
  digitalWrite(LeftMotorDirPin2B,HIGH);
  analogWrite(speedPinLB,speed);
}
 
void stop_Stop()    //Stop
{
  analogWrite(speedPinLB,0);
  analogWrite(speedPinRB,0);
  analogWrite(speedPinL,0);
  analogWrite(speedPinR,0);
}