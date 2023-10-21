/* mbed Microcontroller Library
 * Copyright (c) 2023 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "../../KRAI_Library/encoderHAL/encoderHAL.h"
#include "../../KRAI_Library/encoderHAL/encoderMspInitF4.h"
#include "../../KRAI_Library/pinout/F446RE_MASTER_2022.h"
#include "../../KRAI_Library/encoderKRAI/encoderKRAI.h"


#define PPR_CUI 2050

static BufferedSerial serial_port(USBTX, USBRX, 115200);
FileHandle *mbed::mbed_override_console(int fd)
{
    return &serial_port;
}


encoderHAL enc(F446RE_MASTER_ENCODER_A_D_A, F446RE_MASTER_ENCODER_A_D_B, PPR_CUI, Encoding::X4_ENCODING);

const float kel = 2 * 3.14f * 2.9f;
float pulse, rotation;
float s;
uint32_t curr=us_ticker_read();

int main()
{   
    while (true)
    {
        // Coba
        if(us_ticker_read() - curr > 50000){
            pulse = (float)(enc.getPulses());
            rotation = pulse / (PPR_CUI * 4);
            s = rotation * kel;
            printf("PULSE: %d DISTANCE: %.3f cm\n", enc.getPulses(), s);
            curr=us_ticker_read();
        }
    }
}