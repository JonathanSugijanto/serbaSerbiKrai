#include "mbed.h"
#include "Configs/ConfigurationPin.h"
#include "Configs/Constants.h"
#include "../../KRAI_Library/encoderKRAI/encoderKRAI.h"
#include "../../KRAI_Library/Motor/Motor.h"
#include "../../KRAI_Library/pidLo/pidLo.h"
#include "../../KRAI_Library/CMPS12_KRAI/CMPS12_KRAI.h"
#include "../../KRAI_Library/JoystickPS3/JoystickPS3.h"
#include "ExtraLibs/MovingAverage/MovingAverage.h"
#include "ExtraLibs/FiveWheel/FiveWheel.h"
// #include "../../KRAI_Library/Pinout/F446RE_MASTER_2022.h"

// INITIALIZE SERIAL USING PRINTF 
static BufferedSerial serial_port(PA_9, PA_10, 115200);
FileHandle *mbed::mbed_override_console(int fd){
    return &serial_port;
}

// JoystickPS3 ps3(UART_STICK_TX, UART_STICK_RX);

Motor baseMotorFL(BASE_MOTOR_FL_PWM, BASE_MOTOR_FL_FOR, BASE_MOTOR_FL_REV);
Motor baseMotorFR(BASE_MOTOR_FR_PWM, BASE_MOTOR_FR_FOR, BASE_MOTOR_FR_REV);
Motor baseMotorBL(BASE_MOTOR_BL_PWM, BASE_MOTOR_BL_FOR, BASE_MOTOR_BL_REV);
Motor baseMotorBR(BASE_MOTOR_BR_PWM, BASE_MOTOR_BR_FOR, BASE_MOTOR_BR_REV);
Motor baseMotorMID(BASE_MOTOR_MID_PWM, BASE_MOTOR_MID_FOR, BASE_MOTOR_MID_REV);

encoderKRAI encoderBaseFL(ENCODER_BASE_MOTOR_FL_CHA, ENCODER_BASE_MOTOR_FL_CHB, PPR_FL / 2, X2_ENCODING);
encoderKRAI encoderBaseFR(ENCODER_BASE_MOTOR_FR_CHA, ENCODER_BASE_MOTOR_FR_CHB, PPR_FR / 2, X2_ENCODING);
encoderKRAI encoderBaseBR(ENCODER_BASE_MOTOR_BR_CHA, ENCODER_BASE_MOTOR_BR_CHB, PPR_BR / 2, X2_ENCODING);
encoderKRAI encoderBaseBL(ENCODER_BASE_MOTOR_BL_CHA, ENCODER_BASE_MOTOR_BL_CHB, PPR_BL / 2, X2_ENCODING);
encoderKRAI encoderBaseMID(ENCODER_BASE_MOTOR_MID_CHA, ENCODER_BASE_MOTOR_MID_CHB, PPR_MID / 2, X2_ENCODING);

pidLo pidLoMotorFL(BASE_FL_KP, BASE_FL_KI, BASE_FL_KD, SAMP_PID_BASE_MOTOR_MS, 1.0, 0, 1000, 1000);
pidLo pidLoMotorFR(BASE_FR_KP, BASE_FR_KI, BASE_FR_KD, SAMP_PID_BASE_MOTOR_MS, 1.0, 0, 1000, 1000);
pidLo pidLoMotorBL(BASE_BL_KP, BASE_BL_KI, BASE_BL_KD, SAMP_PID_BASE_MOTOR_MS, 1.0, 0, 1000, 1000);
pidLo pidLoMotorBR(BASE_BR_KP, BASE_BR_KI, BASE_BR_KD, SAMP_PID_BASE_MOTOR_MS, 1.0, 0, 1000, 1000);
pidLo pidLoMotorMID(BASE_MID_KP, BASE_MID_KI, BASE_MID_KD, SAMP_PID_BASE_MOTOR_MS, 1.0, 0, 1000, 1000);

FiveWheel Fivewheel();

float testSpeed=0.0f;

uint32_t samplingBaseEncoder=0;

int main(){
    // ps3.idle();   
    // ps3.setup();
    FiveWheel.encoderMotorSamp();
    while(true){
        // ps3.olah_data();
        // ps3.baca_data();

        if (serial_port.readable())
        {
            scanf("%f", &testSpeed);
            FiveWheel.testSpeed(testSpeed);
        }

        if(us_ticker_read() - samplingBaseEncoder > SAMP_BASE_MOTOR_ENCODER_US){
            FiveWheel.encoderMotorSamp();
            samplingBaseEncoder = us_ticker_read();
        }

    }
    return 0;
}