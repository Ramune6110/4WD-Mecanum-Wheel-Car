#include "mbed.h"              

Serial pc(USBTX, USBRX); // tx, rx

int main() {
    pc.baud(115200);
    
    while(1) {
        
        pc.printf("Hello World!\n");
        wait_ms(10);
    }
}