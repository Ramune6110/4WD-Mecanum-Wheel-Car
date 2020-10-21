#include "mbed.h"
#include "Serial.h"

Serial pc(USBTX, USBRX); // tx, rx

BusIn encoder_bus_left(PA_5, PA_6); //Encoder (LSB to MSB)
BusIn encoder_bus_right(PA_7, PB_6); //Encoder (LSB to MSB)

int encoder_value_left  = 0;
int encoder_value_right = 0;
int table[16] = {0, 1, -1, 0,  -1, 0, 0, 1,  1, 0, 0, -1,  0, -1, 1, 0};

// ロータリーエンコーダの1回転あたりのパルス数
float rotary_encoder_resolution = 120.0f;

// 現時刻でのロータリーエンコーダの角度を保持
float rotation_angle_left  = 0.0f;
float rotation_angle_right = 0.0f;

// 前時刻でのロータリーエンコーダの角度を保持
float pre_rotation_angle_left = 0.0f;
float pre_rotation_angle_right = 0.0f;

// 左右の回転速度
float rotation_angular_velocity_left  = 0.0f;
float rotation_angular_velocity_right = 0.0f;

// 時間計測
Timer t;
float dt = 0.0f;
float current_time = 0.0f;
float last_time = 0.0f;

int main() {   
    /***********************************************************************
     * Initialization
     **********************************************************************/
    pc.baud(115200);
    t.start();
    current_time = t.read();
    last_time = t.read();
    
    while(1)
    {
        /***********************************************************************
        * Timer Count
        **********************************************************************/
        current_time = t.read();
        dt = current_time - last_time;

        /***********************************************************************
        * Encoder Value
        **********************************************************************/
        static int code_left; 
        static int code_right; 
        //check the movement
        code_left  = ( (code_left<<2) +  int(encoder_bus_left) ) & 0xf ;
        code_right  = ( (code_right<<2) +  int(encoder_bus_right) ) & 0xf ;
        //update the encoder value
        int value_left  = -1 * table[code_left];
        int value_right  = -1 * table[code_right];
        encoder_value_left += value_left;
        encoder_value_right += value_right;
        
        /**
        ***********************************************************************
        * Odometry
        ***********************************************************************
        */
        //-------------------------------------------
        // タイヤの回転角
        //-------------------------------------------
        rotation_angle_left  = (float) (encoder_value_left) * (2.0f * 3.14f)/(4.0f * rotary_encoder_resolution);
        rotation_angle_right = (float) (encoder_value_right) * (2.0f * 3.14f)/(4.0f * rotary_encoder_resolution);

        //-------------------------------------------
        // タイヤの回転速度
        //-------------------------------------------
        rotation_angular_velocity_left  = (rotation_angle_left - pre_rotation_angle_left) / dt;
        rotation_angular_velocity_right = (rotation_angle_right - pre_rotation_angle_right) / dt;
    
        // display encoder value
        //pc.printf("encoder_value_left = %d\r\n", encoder_value_left);
        //pc.printf("encoder_value_right = %d\r\n", encoder_value_right);
        //pc.printf("left_velocity = %f\r\n", rotation_angular_velocity_left);
        //pc.printf("right_velocity = %f\r\n", rotation_angular_velocity_right);
        //pc.printf("rotation_angle_left = %f\r\n", rotation_angle_left);
        //pc.printf("rotation_angle_right = %f\r\n", rotation_angle_right);
        //pc.printf("pre_rotation_angle_left = %f\r\n", pre_rotation_angle_left);
        //pc.printf("pre_rotation_angle_right = %f\r\n", pre_rotation_angle_right);
        pc.printf("The time taken was %f seconds\n", dt);

        // preserve
        pre_rotation_angle_left  = rotation_angle_left;
        pre_rotation_angle_right = rotation_angle_right;
        last_time = current_time;

        // wait 
        wait_us(10000);    
    }    
}