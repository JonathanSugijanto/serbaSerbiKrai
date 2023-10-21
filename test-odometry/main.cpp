#include "mbed.h"
#include "../../KRAI_Library/encoderHAL/encoderHAL.h"
#include "../../KRAI_Library/encoderKRAI/encoderKRAI.h"
#include "../../KRAI_Library/CMPS12_KRAI/CMPS12_KRAI.h"
#include "../../KRAI_Library/Pinout/F446RE_MASTER_2022.h"
#include <cmath>

#define CHA_X F446RE_MASTER_ENCODER_A_A_A
#define CHB_X F446RE_MASTER_ENCODER_A_A_B
#define CHA_Y F446RE_MASTER_ENCODER_A_D_A
#define CHB_Y F446RE_MASTER_ENCODER_A_D_B
#define PPR_X 400
#define PPR_Y 2100
#define PI 3.14159265358979

#define cmps_SDA PB_9 //SDA 1 F446RE
#define cmps_SCL PB_8 //SCL 1 F446RE 
#define cmps_ADD 0xC0 //default address CMPS12

// INITIALIZE SERIAL USING PRINTF 
static BufferedSerial serial_port(USBTX, USBRX, 115200);
FileHandle *mbed::mbed_override_console(int fd){
    return &serial_port;
}

encoderKRAI encCuiX(CHB_X, CHA_X, PPR_X, Encoding::X4_ENCODING);
encoderKRAI encCuiY(CHB_Y, CHA_Y, PPR_Y, Encoding::X4_ENCODING);
CMPS12_KRAI cmps(cmps_SDA, cmps_SCL, cmps_ADD);

class RobotState
{
private:
    encoderKRAI *encCuiX;
    encoderKRAI *encCuiY;
    CMPS12_KRAI *cmps;

    float x, y, theta;
    float L = 0.165; // Jarak dari pusat koordinat robot ke cui
    float B = 0.19; // Jarak dari cuiX ke CuiY
    float r = (58 / 2) / 1000.0f; // Jari-jari cui
    float tempPulses = 0.0f;

public:
    RobotState(encoderKRAI *encCuiX, encoderKRAI *encCuiY, CMPS12_KRAI *cmps)
    {
        this->encCuiX = encCuiX;
        this->encCuiY = encCuiY;
        this->cmps = cmps;
        this->x = 0;
        this->y = 0;
        this->theta = this->cmps->getAngle() / 10.0f;
    }

    float getX(){ return this->x; }
    float getY(){ return this->y; }
    float getTheta(){ return this->theta; }

    void updatePose()
    {
        float deltaTheta1 = this->cmps->getAngle() / 10.0f - this->theta;
        float deltaTheta2 = 360 - std::fabs(deltaTheta1);
        float deltaTheta = (std::fabs(deltaTheta1) < deltaTheta2) ? std::fabs(deltaTheta1) : deltaTheta2;
        if (deltaTheta1 > 0)
        {
            deltaTheta = deltaTheta * (-1);
        }
        deltaTheta = deltaTheta * PI / 180.0f; // Ubah degree to rad

        float deltaX = (2 * PI * r /(float)PPR_X) * this->encCuiX->getPulses() + L * deltaTheta;
        float deltaY = (2 * PI * r /(float)PPR_Y) * this->encCuiY->getPulses() + B * deltaTheta;

        this->x = this->x + cos(deltaTheta) * deltaX - sin(deltaTheta) * deltaY;
        this->y = this->y + sin(deltaTheta) * deltaX + cos(deltaTheta) * deltaY;

        this->theta = this->cmps->getAngle() / 10.0f; // Degree
    }

    void resetCmps()
    {
        this->cmps->compassResetOffsetValue();
    }
    void resetCui()
    {
        this->encCuiX->reset();
        this->encCuiY->reset();
    }

    void sensorRead()
    {
        printf("Cui_X: %d Cui_Y: %d Yaw: %d\n", this->encCuiX->getPulses(), this->encCuiY->getPulses(), this->cmps->getAngle());
        // printf("accelX: %d\n", this->cmps->getAccelX());
    }

    void printPose()
    {
        printf("X: %f Y: %f yaw: %f\n", this->x, this->y, this->theta);
    }
};

/*
    1.000.000 mikrosecond = 1 second
    1.000 mikrosecond = 1 miliSecond
*/
uint32_t samplingPose = 10 * 1000; // mikrosecond
uint32_t samplingPrintHasil = 10 * 1000; // 10 miliSecond
uint32_t timeElapsed = us_ticker_read();
uint32_t timeElapsedForPrint = us_ticker_read();

int main()
{
    RobotState poseState(&encCuiX, &encCuiY, &cmps);
    poseState.resetCui();
    poseState.resetCmps();

    while (1)
    {
        if (us_ticker_read() - timeElapsed > samplingPose)
        {
            poseState.updatePose();
            poseState.resetCui();
            timeElapsed = us_ticker_read();
        }

        if (us_ticker_read() - timeElapsedForPrint > samplingPrintHasil)
        {
            poseState.printPose(); // Print Pembacaan nilai X, Y, dan Yaw saat ini
            // poseState.sensorRead();
            timeElapsedForPrint = us_ticker_read();
        }
    }

    return 0;
}