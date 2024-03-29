#include "mbed.h"
#include "GY80/GY80.h"

#define timeSampling 100000
#define timeSamp 1250

GY80 gy(PB_9,PB_8,timeSamp);

float Speed[3];
float Angle[3];
float Gyro[3];
float Mag[3];
float Angle1[3];

float tot;

uint32_t timer1=0;
uint32_t timer2=0;

static BufferedSerial serial_port(USBTX, USBRX, 115200);
FileHandle *mbed::mbed_override_console(int fd){
    return &serial_port;
}

int16_t gyro_raw[3];

int main()
{
    // gy.set_b_verbose(true);
    while(1){
        if(us_ticker_read() - timer1 > timeSampling){
            // gy.Compass_rel(Angle);
            // gy.Compass(Angle);
            gy.simple_Compass(Angle);
            // gy.ex_Compass(Speed, Angle);
            // gy.ex1_Compass(Angle);
            // gy.Compass_No_Gyro(Angle);
            // printf("Sx: %.1f Sy: %.1f Sz: %.1f ", Speed[0], Speed[1], Speed[2]);
            printf("Roll: %.1f Pitch: %.1f Yaw: %.1f", Angle[0], Angle[1], Angle[2]);
            // printf("RollG: %.1f RollM: %.1f PitchG: %.1f PitchM: %.1f YawG: %.1f YawM: %.1f", Angle[0], Angle1[0], Angle[1], Angle1[1], Angle[2], Angle1[2]);

            // gy.Read_Magn(Gyro);
            // gy.Read_Raw_Gyro(gyro_raw);
            // printf("%d %d %d", gyro_raw[0], gyro_raw[1], gyro_raw[2]);

            // gy.Read_Gyro(Gyro);
            // for(int i = 0; i<3 ; i++){
            //     Gyro[i] *= RAD2DEG;
            // }
            // tot=sqrt(float(Gyro[0]*Gyro[0]+Gyro[1]*Gyro[1]+Gyro[2]*Gyro[2]));
            // printf("x: %d Y: %d Z: %d tot: %.2f", Gyro[0], Gyro[1], Gyro[2], tot);
            // tot=sqrt(Gyro[0]*Gyro[0]+Gyro[1]*Gyro[1]+Gyro[2]*Gyro[2]);
            // printf("x: %.2f Y: %.2f Z: %.2f tot: %.2f", Gyro[0], Gyro[1], Gyro[2], tot);

            printf("\n");

            timer1=us_ticker_read();
        }

        if(us_ticker_read() - timer2 > timeSamp){
            // gy.ex_Sampling();
            // gy.ex1_Sampling();
            // gy.Sampling();
            gy.simple_Sampling();
            timer2 = us_ticker_read();
        }
    }

    return 0;
}