
#include "mbed.h"
#include "GY80/GY80.h"

#define timeSampling 100000

GY80 gy(PB_9,PB_8);

float Acc[3];
float Gyro[3];
float Mag[3];

int16_t raw_Mag[3];

float MagTot, Zangle;

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
            // gy.Read_Accel(Acc);
            // gy.Read_Gyro(Gyro);
            gy.Read_Magn(Mag);
            // gy.Read_Raw_Magn(raw_Mag);

            // for(int i=0; i<3; i++){
            //     Mag[i]=(float)raw_Mag[i];
            // }

            // printf("AccX: %.2f AccY: %.2f AccZ: %.2f ", Acc[0], Acc[1], Acc[2]);
            // printf("GyroX: %.2f GyroY: %.2f GyroZ: %.2f ", Gyro[0], Gyro[1], Gyro[2]);
            MagTot=sqrt(Mag[0]*Mag[0]+Mag[1]*Mag[1]+Mag[2]*Mag[2]);
            // printf("MagX: %.2f MagY: %.2f MagZ: %.2f Mag: %.2f", Mag[0], Mag[1], Mag[2], MagTot);
            // MagTot=sqrt((float)(raw_Mag[0]*raw_Mag[0]+raw_Mag[1]*raw_Mag[1]+raw_Mag[2]*raw_Mag[2]));
            // printf("MagX: %d MagY: %d MagZ: %d Mag: %.2f", raw_Mag[0], raw_Mag[1], raw_Mag[2], MagTot);

            Zangle = gy.No_Tilt_Compass();
            printf("Zangle: %.2f Mag: %.2f MagX: %.2f MagY: %.2f MagZ: %.2f", Zangle, MagTot, Mag[0], Mag[1], Mag[2]);
            printf("\n");

            // gy.Cal_Magn();

            timer1=us_ticker_read();
        }
    }

    return 0;
}