/* mbed Microcontroller Library
 * Copyright (c) 2023 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "../../KRAI_Library/encoderHAL/encoderHAL.h"
#include "../../KRAI_Library/encoderHAL/encoderMspInitF4.h"
#include "../../KRAI_Library/pinout/F446RE_MASTER_2022.h"
#include "../../KRAI_Library/encoderKRAI/encoderKRAI.h"

#include "../../KRAI_Library/CMPS12_KRAI/CMPS12_KRAI.h"


#define PPR_CUI 100

static BufferedSerial serial_port(USBTX, USBRX, 115200);
FileHandle *mbed::mbed_override_console(int fd)
{
    return &serial_port;
}


encoderKRAI enc1(F446RE_MASTER_ENCODER_A_C_A, F446RE_MASTER_ENCODER_A_C_B, PPR_CUI, Encoding::X4_ENCODING);
encoderKRAI enc2(F446RE_MASTER_ENCODER_A_D_A, F446RE_MASTER_ENCODER_A_D_B, PPR_CUI, Encoding::X4_ENCODING);

const float kel = 2 * 3.14f * 2.9f;
float pulse1, rotation1;
float s1;
float pulse2, rotation2;
float s2;
uint32_t curr=us_ticker_read();

#define cmps_SDA PB_9 //SDA 1
#define cmps_SCL PB_8 //SCL 1
#define cmps_ADD 0xC0
CMPS12_KRAI cmps(cmps_SDA, cmps_SCL, cmps_ADD);

int main()
{   
    cmps.compassResetOffsetValue();
    while (true)
    {
        // Coba
        if(us_ticker_read() - curr > 50000){
            pulse1 = (float)(enc1.getPulses());
            rotation1 = pulse1 / (PPR_CUI * 4);
            s1 = rotation1 * kel;
            printf("PULSE: %d DISTANCE: %.3f cm", enc1.getPulses(), s1);

            pulse2 = (float)(enc2.getPulses());
            rotation2 = pulse2 / (PPR_CUI * 4);
            s2 = rotation2 * kel;
            printf("PULSE: %d DISTANCE: %.3f cm", enc2.getPulses(), s2);

            cmps.compassUpdateValue();
            printf("cmpsVal: %.2f Angle: %d Pitch: %d Roll: %d \n ", cmps.compassValue(), cmps.getAngle(), cmps.getPitch(), cmps.getRoll());

            curr=us_ticker_read();
        }
    }
}