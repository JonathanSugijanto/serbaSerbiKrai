#include "mbed.h"
#include "../../modifKRAI_Library/CMPS12_KRAI/CMPS12_KRAI.h"
#include "../../KRAI_Library/Pinout/F446RE_MASTER_2022.h"

#ifndef PI
#define PI 3.14159265359f
#endif

#define timeSampling 500
float deltaT = (float)timeSampling/1000000.0f;

static BufferedSerial serial_port(USBTX, USBRX, 115200);
FileHandle *mbed::mbed_override_console(int fd){
    return &serial_port;
}

#define cmps_SDA PB_9 //SDA 1
#define cmps_SCL PB_8 //SCL 1
#define cmps_ADD 0xC0
CMPS12_KRAI cmps(cmps_SDA, cmps_SCL, cmps_ADD);

DigitalOut led1(LED1);
// DigitalOut sda_pin(cmps_SDA);
// DigitalOut scl_pin(cmps_SCL);
uint32_t curr=us_ticker_read();
uint32_t curr1=us_ticker_read();
float nyoba=deltaT;

float convertToHuman(float dataIn){
    return dataIn/100.0f;
}
float speedx=0;
float posx=0;
int16_t lastAccelx=0;
float lastSpeedx=0;

int16_t ax0=0;
int16_t ay0=0;
int16_t az0=0;

void calibrateGo(){
    int bankAx=0;
    int bankAy=0;
    int bankAz=0;
    for(int i=0; i<100; i++){
        bankAx+=cmps.getAccelX();
        bankAy+=cmps.getAccelY();
        bankAz+=cmps.getAccelZ();
    }
    ax0=bankAx/100;
    ay0=bankAy/100;
    az0=bankAz/100;
}

// Ticker integTick;
void integrateSamp(){
    int16_t currAccX= cmps.getAccelX() - ax0;
    lastSpeedx=speedx;
    speedx+=(float)(currAccX+lastAccelx)/2.0f * deltaT;
    lastAccelx=currAccX;
    posx+=(speedx+lastSpeedx)/2.0f * deltaT;
}

//acceleration in respect to world
float axw, ayw, azw;
float axb1, ayb1, azb1;
void worldAccelTransform(){
    //if front=bawah tulisan cmps12 output and input in cm/s^2
    //ubah konvensi ikutin file:///C:/Users/Dell/Downloads/Spong-RobotmodelingandControl%20(1).pdf hlm 62
    //tambahan referensi https://huskiecommons.lib.niu.edu/cgi/viewcontent.cgi?article=1032&context=allgraduate-thesesdissertations 
    float axb= (float)(- cmps.getAccelX() + ax0); //acceleration in respect to cmps12
    float ayb= (float)(- cmps.getAccelY() + ay0);
    float azb= (float)(- cmps.getAccelZ() + az0);

    float roll= - (float)cmps.getAngle() * PI / 1800.0f;
    float pitch= - (float)cmps.getRoll() * PI / 180.0f;
    float yaw = - (float)cmps.getPitch() * PI / 180.0f;

    float cRoll = cos(roll);
    float sRoll = sin(roll);
    float cPitch = cos(pitch);
    float sPitch = sin(pitch);
    float cYaw = cos(yaw);
    float sYaw = sin(yaw);

    axw = (cRoll*cPitch)*axb + (-sRoll*cYaw+cRoll*sPitch*sYaw)*ayb + (sRoll*sYaw+cRoll*sPitch*cYaw)*azb;
    ayw = (sRoll*cPitch)*axb + (cRoll*cYaw+sRoll*sPitch*sYaw)*ayb + (-cRoll*sYaw+sRoll*sPitch*cYaw)*azb;
    azw = (-sPitch)*axb + (cPitch*sYaw)*ayb + (cPitch*cYaw)*azb;

    // axb1 = (cRoll*cPitch)*axw + (sRoll*cPitch)*ayw + (-sPitch)*azw;
    // ayb1 = (-sRoll*cYaw+cRoll*sPitch*sYaw)*axw + (cRoll*cYaw+sRoll*sPitch*sYaw)*ayw + (cPitch*sYaw)*azw;
    // azb1 = (sRoll*sYaw+cRoll*sPitch*cYaw)*axw + (-cRoll*sYaw+sRoll*sPitch*cYaw)*ayw + (cPitch*cYaw)*azw;
}

float t=0.0f;

int main()
{
    led1=1;
    // sda_pin=1;
    // scl_pin=1;
    cmps.compassResetOffsetValue();
    // calibrateGo();
    // calibrateGo1();
    // integTick.attach_us(&integrateSamp, timeSampling);
    while(true){
        if ((us_ticker_read() - curr) > 50000) // batas us_ticker di blue pill
        {
            // led1 = !led1;
            cmps.compassUpdateValue();
            worldAccelTransform();
            // printf("%.2f ", nyoba);
            // printf("AccX: %.5f speedX: %.5f pos_x_cm: %.2f\n ", convertToHuman((float)cmps.getAccelX() - ax0), convertToHuman(speedx), posx);
            // float Accel = sqrtf((float)(cmps.getAccelX() * cmps.getAccelX() + cmps.getAccelY() * cmps.getAccelY() + cmps.getAccelZ() * cmps.getAccelZ()));
            // printf("Accel_x: %d Accel_y: %d Accel_z: %d Accel: %.2f\n", cmps.getAccelX() - ax0, cmps.getAccelY() - ay0, cmps.getAccelZ() - az0, Accel);
            // printf("yaw: %d pitch: %d roll: %d\n", cmps.getAngle(), cmps.getPitch(), cmps.getRoll());

            
            printf("Accel_x: %d Accel_y: %d Accel_z: %d axw: %.2f ayw: %.2f azw: %.2f roll: %d pitch: %d yaw: %d\n", - cmps.getAccelX(), - cmps.getAccelY(), - cmps.getAccelZ(), axw, ayw, azw, -cmps.getAngle(), -cmps.getRoll(), -cmps.getPitch());
            curr = us_ticker_read();
        }
        if(us_ticker_read() - curr1 >timeSampling){
            integrateSamp();
            curr1 = us_ticker_read();
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