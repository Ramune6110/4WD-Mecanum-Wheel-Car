#include "mbed.h"
#include "Serial.h"

Serial pc(USBTX, USBRX); // tx, rx

BusIn encoder_bus_left(PA_5, PA_6); //Encoder (LSB to MSB)
BusIn encoder_bus_right(PA_7, PB_6); //Encoder (LSB to MSB)

int encoder_value_left  = 0;
int encoder_value_right = 0;
int table[16] = {0, 1, -1, 0,  -1, 0, 0, 1,  1, 0, 0, -1,  0, -1, 1, 0};

int main() {   
    /***********************************************************************
     * Initialization
     **********************************************************************/
    //-------------------------------------------
    // System initialization
    //-------------------------------------------
    pc.baud(115200);
    
    while(1)
    {
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
    
        // display encoder value
        pc.printf("encoder_value_left = %d\r\n", encoder_value_left);
        pc.printf("encoder_value_right = %d\r\n", encoder_value_right);

        // wait 
        wait_ms(10);    
    }    
}