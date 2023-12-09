
#include "mbed.h"
#include "GY80/GY80.h"

#define timeSampling 100000

GY80 gy(PB_9,PB_8);

float Acc[3];
float Gyro[3];
float Mag[3];

int16_t raw_Acc[3];
int16_t raw_Mag[3];

float AccTot, Zangle;

uint32_t timer1=0;

static BufferedSerial serial_port(USBTX, USBRX, 115200);
FileHandle *mbed::mbed_override_console(int fd){
    return &serial_port;
}

int main()
{
    // gy.set_b_verbose(true);
    printf("finished initializing");
    while(1){
        if(us_ticker_read() - timer1 > timeSampling){
            gy.Read_Accel(Acc);
            // gy.Read_Raw_Accel(raw_Acc);
            // gy.Read_Gyro(Gyro);
            // gy.Read_Accn(Acc);
            // gy.Read_Raw_Accn(raw_Acc);

            // for(int i=0; i<3; i++){
            //     Acc[i]=(float)raw_Acc[i];
            // }

            // printf("AccX: %.2f AccY: %.2f AccZ: %.2f ", Acc[0], Acc[1], Acc[2]);
            // printf("GyroX: %.2f GyroY: %.2f GyroZ: %.2f ", Gyro[0], Gyro[1], Gyro[2]);
            AccTot=sqrt(Acc[0]*Acc[0]+Acc[1]*Acc[1]+Acc[2]*Acc[2]);
            printf("AccX: %.2f AccY: %.2f AccZ: %.2f Acc: %.2f", Acc[0], Acc[1], Acc[2], AccTot);
            // AccTot=sqrt((float)(raw_Acc[0]*raw_Acc[0]+raw_Acc[1]*raw_Acc[1]+raw_Acc[2]*raw_Acc[2]));
            // printf("AccX: %d AccY: %d AccZ: %d Acc: %.2f", raw_Acc[0], raw_Acc[1], raw_Acc[2], AccTot);

            // printf("Zangle: %.2f Acc: %.2f AccX: %.2f AccY: %.2f AccZ: %.2f", Zangle, AccTot, Acc[0], Acc[1], Acc[2]);
            printf("\n");

            // gy.Cal_Accn();

            timer1=us_ticker_read();
        }
    }

    return 0;
}