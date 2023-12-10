#include "mbed.h"
#include "GY80/GY80.h"

#define timeSampling 100000

GY80 gy(PB_9,PB_8);

float Angle[3];

uint32_t timer1=0;

static BufferedSerial serial_port(USBTX, USBRX, 115200);
FileHandle *mbed::mbed_override_console(int fd){
    return &serial_port;
}

int main()
{
    while(1){
        if(us_ticker_read() - timer1 > timeSampling){
            gy.Compass_rel(Angle);
            printf("Roll: %.1f Pitch: %.1f Yaw: %.1f", Angle[0], Angle[1], Angle[2]);
            printf("\n");

            timer1=us_ticker_read();
        }
    }

    return 0;
}