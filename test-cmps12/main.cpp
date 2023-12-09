#include "mbed.h"
#include "../../KRAI_Library/CMPS12_KRAI/CMPS12_KRAI.h"
#include "../../KRAI_Library/Pinout/F446RE_MASTER_2022.h"

static BufferedSerial serial_port(USBTX, USBRX, 115200);
FileHandle *mbed::mbed_override_console(int fd){
    return &serial_port;
}

#define cmps_SDA PB_9 //SDA 1
#define cmps_SCL PB_8 //SCL 1
// #define cmps_SDA PB_3 //SDA 2
// #define cmps_SCL PB_10 //SCL 2
#define cmps_ADD 0xC0
CMPS12_KRAI cmps(cmps_SDA, cmps_SCL, cmps_ADD);

DigitalOut led1(LED1);
// DigitalOut sda_pin(cmps_SDA);
// DigitalOut scl_pin(cmps_SCL);
uint32_t curr=us_ticker_read();
uint32_t curr1=us_ticker_read();

int main()
{
    led1=1;
    // sda_pin=1;
    // scl_pin=1;
    cmps.compassResetOffsetValue();
    while(true){
        if ((us_ticker_read() - curr) > 50000) // batas us_ticker di blue pill
        {
            // led1 = !led1;
            cmps.compassUpdateValue();
            printf("cmpsVal: %.2f Angle: %d Pitch: %d Roll: %d \n ", cmps.compassValue(), cmps.getAngle(), cmps.getPitch(), cmps.getRoll());
            curr = us_ticker_read();
        }
        // if ((us_ticker_read() - curr1) > 3000000) // batas us_ticker di blue pill
        // {
        //     sda_pin = !sda_pin;
        //     scl_pin = !scl_pin;
        //     curr1 = us_ticker_read();
        // }
    }
    return 0;
}