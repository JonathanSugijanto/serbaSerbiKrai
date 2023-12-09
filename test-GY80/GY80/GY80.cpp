#include "GY80.h"

GY80::GY80() : Wire( SDA,SCL)
{
    Wire.frequency(I2C_FREQ);
    Accel_Init();
    Gyro_Init();
    Magn_Init();
}
GY80::GY80(PinName sda_pin, PinName scl_pin) : Wire( sda_pin, scl_pin)
{
    Wire.frequency(I2C_FREQ);
    Accel_Init();
    Gyro_Init();
    Magn_Init();
}
GY80::~GY80()
{
}
void GY80::wait_ms(int t)
{
    wait_us(1000*t);
}
void GY80::Accel_Init()
{    
    byte data[2];
    data[0] = 0x2D; // Power register
    data[1] = 0x08; //Measurement mode
    Wire.write(ACCEL_ADDRESS, data, 2);
    wait_ms(1);

    data[0] = 0x31; // Data format register
    data[1] = 0x08; //Set to full resolution
    Wire.write(ACCEL_ADDRESS, data, 2);
    wait_ms(1);

    // 
    data[0] = 0x2C; // Rate
    data[1] = 0x0D; //Set to 800Hz, normal operation, 0x0A 100hz 
    Wire.write(ACCEL_ADDRESS, data, 2);
    wait_ms(1);
}

void GY80::Gyro_Init()
{
    byte data[2];

    data[0] = 0x20; //L3G4200D_CTRL_REG1
    data[1] = 0xCF; // normal power mode, all axes enable, 8:20 9:25 A:50 B:110 
    Wire.write(GYRO_ADDRESS, data, 2);
    wait_ms(1);


    data[0] = 0x23; // L3G4200D_CTRL_REG4
    data[1] = 0x20; //2000 dps full scale 
    Wire.write(GYRO_ADDRESS, data, 2);
    wait_ms(1);


    data[0] = 0x24; // L3G4200D_CTRL_REG5
    data[1] = 0x02; //Low Pass Filter
    Wire.write(GYRO_ADDRESS, data, 2);
}

void GY80::Magn_Init()
{   
    byte data[2];
    data[0] = 0x00;
    //note: bit 0-1: measurement mode (00 normal)
    //      bit 2-4: Data output rate for continuous mode (100 for 15Hz (default), 110 for 75Hz (fastest))
    //      bit 5-6: "NUmber of samples averaged per measurement output" (00 for 1 (default), 01 for 2, 10 for 4, 11 for 8)
    data[1] = 0x50; // 01010000 Set Moving Average of the last 4 data with 15Hz data rate
    Wire.write(MAGN_ADDRESS_W, data, 2);
    wait_ms(1);

    data[0] = 0x01;
    data[1] = 0x00; // 00000000 Set resolution to +-0.88 Ga reccomended range or 1370 LSb/Ga (note: default: +-1.3 Ga or 1090 Lsb/Ga)
    Wire.write(MAGN_ADDRESS_W, data, 2);
    wait_ms(1);
    
    data[0] = 0x02;
    data[1] = 0x00; // 00000000 Set continuous mode
    Wire.write(MAGN_ADDRESS_W, data, 2);
    wait_ms(7);

    Read_Raw_Magn(mag); //first data is thrown away
    wait_ms(7);
}

float GY80::No_Tilt_Compass(){
    float f_mag[3];
    Read_Magn(f_mag);
    float Zangle = - atan2(f_mag[1], f_mag[0]);
    if(Zangle<0) Zangle+=2.0f * M_PI;
    Zangle*=180.0f/M_PI;
    return Zangle;
}

void GY80::Read_Accel(float* accel_v)
{
    byte buff[6];
    buff[0] = 0x32; // Send address to read from
    Wire.write(ACCEL_ADDRESS, buff, 1);

    int accel[3];
    if (Wire.read(ACCEL_ADDRESS, buff,6) == 0)  // All bytes received?
    {
        accel[0] = (short) ((uint16_t) buff[1] << 8 | buff[0]);
        accel[1] = (short) ((uint16_t) buff[3] << 8 | buff[2]);
        accel[2] = (short) ((uint16_t) buff[5] << 8 | buff[4]);
    }
    accel_v[0] = (accel[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
    accel_v[1] = (accel[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
    accel_v[2] = (accel[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;
    
    accel_v[0] = (accel_v[0]/255.0)*1000;
    accel_v[1] = (accel[1]/255.0)*1000;
    accel_v[2] = (accel[2]/255.0)*1000;
    
}


void GY80::Read_Gyro(float* gyro_v)
{
    byte buff[6];

    buff[0] = 0xA8; // 0x28 | (1 << 7) Send address to read from 
    Wire.write(GYRO_ADDRESS, buff, 1);
    // Request 6 bytes
    int gyro[3];
    if (Wire.read(GYRO_ADDRESS, buff,6) == 0)  // All bytes received?
    {
        gyro[0] = (short) ((uint16_t) buff[1] << 8 | buff[0]);
        gyro[1] = (short) ((uint16_t) buff[3] << 8 | buff[2]);
        gyro[2] = (short) ((uint16_t) buff[5] << 8 | buff[4]);
    }
    
    gyro_v[0] = DEG2RAD((gyro[0] - GYRO_X_OFFSET) * GYRO_GAIN_X); 
    gyro_v[1] = DEG2RAD((gyro[1] - GYRO_Y_OFFSET) * GYRO_GAIN_Y);
    gyro_v[2] = DEG2RAD((gyro[2] - GYRO_Z_OFFSET) * GYRO_GAIN_Z);
    
}

void GY80::Read_Magn(float* magn_v)
{
    Read_Raw_Magn(mag);

    magn_v[0] = ((short)mag[0] - MAGN_X_OFFSET) * MAGN_X_SCALE;
    magn_v[1] = ((short)mag[1] - MAGN_Y_OFFSET) * MAGN_Y_SCALE;
    magn_v[2] = ((short)mag[2] - MAGN_Z_OFFSET) * MAGN_Z_SCALE;
}

void GY80::Read_Raw_Magn(int16_t* magn_v)
{
    byte Rbuff[6];

    // Request 6 bytes
    if (Wire.read(MAGN_ADDRESS_R, Rbuff,6) == 0)  // All bytes received?
    {
        magn_v[0] = ((int16_t) Rbuff[0] << 8 | Rbuff[1]); //sumbu x
        magn_v[1] = ((int16_t) Rbuff[4] << 8 | Rbuff[5]); //sumbu y
        magn_v[2] = ((int16_t) Rbuff[2] << 8 | Rbuff[3]); //sumbu z
    }

    //reset read status
    byte Wbuff = 0x03;
    Wire.write(MAGN_ADDRESS_W,&Wbuff,1);
}

void GY80::set_b_verbose(bool debugDisplay){
    b_verbose = debugDisplay;
}

void GY80::Cal_Magn()
{
    byte data[2];

    //positive self-test
    if(!b_verbose) printf("callibrating max value..\n");
    data[0] = 0x00;
    data[1] = 0x71; //8 average, 15 Hz, positive self-test
    Wire.write(MAGN_ADDRESS_W, data, 2);
    wait_ms(1);

    data[0] = 0x01;
    data[1] = 0x40; // gain=2 (820LSb/Ga)
    Wire.write(MAGN_ADDRESS_W, data, 2);
    wait_ms(1);
    
    data[0] = 0x02;
    data[1] = 0x00; // 00000000 Set continuous mode
    Wire.write(MAGN_ADDRESS_W, data, 2);
    wait_ms(7);

    Read_Raw_Magn(mag); //first data is thrown away
    wait_ms(7);
    float MaxMag[3] = {0.0f,0.0f,0.0f};
    for(int i=0; i<100; i++){
        Read_Raw_Magn(mag);
        for(int j=0; j<3; j++){
            MaxMag[j]=(short)mag[j] + MaxMag[j];
        }
        wait_ms(7);
    }
    for(int j=0; j<3; j++){
        MaxMag[j]=MaxMag[j]/100.0f;
    }

    //negative self-test
    if(!b_verbose) printf("callibrating min value..\n");
    data[0] = 0x00;
    data[1] = 0x72; //8 average, 15 Hz, negative self test (72)
    Wire.write(MAGN_ADDRESS_W, data, 2);
    wait_ms(1);

    data[0] = 0x01;
    data[1] = 0x40; // gain=2 (820LSb/Ga)
    Wire.write(MAGN_ADDRESS_W, data, 2);
    wait_ms(1);
    
    data[0] = 0x02;
    data[1] = 0x00; // 00000000 Set continuous mode
    Wire.write(MAGN_ADDRESS_W, data, 2);
    wait_ms(7);

    Read_Raw_Magn(mag); //first data is thrown away
    wait_ms(7);
    float MinMag[3] = {0.0f,0.0f,0.0f};
    for(int i=0; i<100; i++){
        Read_Raw_Magn(mag);
        for(int j=0; j<3; j++){
            MinMag[j]=(short)mag[j] + MinMag[j];
        }
        wait_ms(7);
    }
    for(int j=0; j<3; j++){
        MinMag[j]=MinMag[j]/100.0f;
    }

    //display
    if(!b_verbose){
        printf("Result of callibration\n");
        printf("using 116 uT at X and Y, and 108 uT at Z\n");

        printf("At Gain 2: 820 LSb/Ga\n");
        printf("MinX: %.5f MaxX: %.5f\n", MinMag[0], MaxMag[0]);
        printf("MinY: %.5f MaxY: %.5f\n", MinMag[1], MaxMag[1]);
        printf("MinZ: %.5f MaxZ: %.5f\n", MinMag[2], MaxMag[2]);

        printf("Scaled t0 Gain 0: 1370 LSb/Ga (used by library)\n");
        printf("MinX: %.5f MaxX: %.5f\n", MinMag[0]*1370.0f/820.0f, MaxMag[0]*1370.0f/820.0f);
        printf("MinY: %.5f MaxY: %.5f\n", MinMag[1]*1370.0f/820.0f, MaxMag[1]*1370.0f/820.0f);
        printf("MinZ: %.5f MaxZ: %.5f\n", MinMag[2]*1370.0f/820.0f, MaxMag[2]*1370.0f/820.0f);
    } else{
        printf("MinX: %.5f MaxX: %.5f ", MinMag[0]*1370.0f/820.0f, MaxMag[0]*1370.0f/820.0f);
        printf("MinY: %.5f MaxY: %.5f ", MinMag[1]*1370.0f/820.0f, MaxMag[1]*1370.0f/820.0f);
        printf("MinZ: %.5f MaxZ: %.5f\n", MinMag[2]*1370.0f/820.0f, MaxMag[2]*1370.0f/820.0f);
    }

    //reset configuration
    Magn_Init();
}