/*
 * CARA MENGGUNAKAN TINGGAL COMMENT/UNCOMMENT micon YG DIPAKE
 * Jangan lupa ubah CMakeLists sesuai letak mbed-os Anda
 * 
 * output benar: print int dan float pada serial monitor (115200) tiap 500ms serta blinking LED utk NUCLEO_F446RE
 * 
 * supported micon: ARCH_MAX, NUCLEO_F446RE
 */

//utk ARCH_MAX
// #define micon_is_ARCH_MAX

//utk NUCLEO_F446RE
#define micon_is_NUCLEO_F446RE

#ifdef micon_is_ARCH_MAX
#define USB_TX PA_9
#define USB_RX PA_10
#endif

#ifdef micon_is_NUCLEO_F446RE
#define USB_TX USBTX
#define USB_RX USBRX
#endif

#include "mbed.h"

DigitalOut led1(LED1);
float counting=0.0f;

static BufferedSerial serial_port(USB_TX, USB_RX, 115200);
FileHandle *mbed::mbed_override_console(int fd){
    return &serial_port;
}

int timer1=us_ticker_read();
int now=us_ticker_read();
char letter='a';

int main()
{
    led1=0;
    while(1){
        now=us_ticker_read();
        if (serial_port.readable())
        {
            scanf("%f", &counting);
            // scanf(" %c", &letter);
            // letter=getchar();
        }
        if(now-timer1>50000){
            led1!=led1;
            printf("Hello, %c %.2f\n", letter, counting);
            counting++;
            timer1=now;
        }
    }
    printf("Hello, Mbed!\n");
    return 0;
}