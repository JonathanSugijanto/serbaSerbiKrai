#include "GY80.h"

GY80::GY80(int timeSampling) : Wire( SDA,SCL)
{
    Wire.frequency(I2C_FREQ);
    this->Accel_Init();
    this->Gyro_Init();
    this->Magn_Init();

    this->timeSampling = timeSampling;
    this->set_Angle(0.0f);
}
GY80::GY80(PinName sda_pin, PinName scl_pin, int timeSampling) : Wire( sda_pin, scl_pin)
{
    Wire.frequency(I2C_FREQ);
    this->Accel_Init();
    this->Gyro_Init();
    this->Magn_Init();

    this->timeSampling = timeSampling;
    this->set_Angle(0.0f);
}
GY80::~GY80()
{
}

float GY80::get_Angle(){
    this->Compass_rel(this->compass);
    return this->compass[0];
}

void GY80::set_Angle(float new_angle){
    //just magnetometer-accelerometer contribution
    this->Read_Magn(this->north);
    this->v_unit(this->north, this->north);
    this->Read_Accel(this->up);
    this->v_unit(this->up, this->up);
    this->find_roll_pitch_yaw();
    this->zero_angle_rel = this->compass[0] + new_angle;
}

float GY80::get_Pitch(){
    this->Compass(this->compass);
    return this->compass[1];
}

float GY80::get_Rotation(){
    this->Compass(this->compass);
    return this->compass[2];
}

void GY80::Compass_rel(float* comp){
    this->Compass(comp);
    comp[0] -= this->zero_angle_rel;
    if(comp[0] < 0.0f) comp[0] += 360.0f;
}

void GY80::Accel_Init()
{    
    byte data[2];
    data[0] = 0x2D; // Power register
    data[1] = 0x08; //Measurement mode
    Wire.write(ACCEL_ADDRESS, data, 2);
   this-> wait_ms(1);

    data[0] = 0x31; // Data format register
    data[1] = 0x08; //Set to full resolution
    Wire.write(ACCEL_ADDRESS, data, 2);
    this->wait_ms(1);

    // 
    data[0] = 0x2C; // Rate
    data[1] = 0x0D; //Set to 800Hz, normal operation, 0x0A 100hz 
    Wire.write(ACCEL_ADDRESS, data, 2);
    this->wait_ms(1);
}

void GY80::Gyro_Init()
{
    byte data[2];

    data[0] = 0x20; //L3G4200D_CTRL_REG1
    data[1] = 0xCF; // normal power mode, all axes enable, 800Hz data rate (with LPF1: 93Hz and LPF2: 30Hz) 8:20 9:25 A:50 B:110 
    Wire.write(GYRO_ADDRESS, data, 2);
    this->wait_ms(1);


    data[0] = 0x23; // L3G4200D_CTRL_REG4
    data[1] = 0x20; //00100000, note: 00xx0000 with xx being the user selectable full scale (00: 250 dps, 01: 500 dps, 1x: 2000 dps)
    Wire.write(GYRO_ADDRESS, data, 2);
    this->wait_ms(1);


    data[0] = 0x24; // L3G4200D_CTRL_REG5
    data[1] = 0x02; // Low Pass Filter
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
    this->wait_ms(1);

    data[0] = 0x01;
    data[1] = 0x00; // 00000000 Set resolution to +-0.88 Ga reccomended range or 1370 LSb/Ga (note: default: +-1.3 Ga or 1090 Lsb/Ga)
    Wire.write(MAGN_ADDRESS_W, data, 2);
    this->wait_ms(1);
    
    data[0] = 0x02;
    data[1] = 0x00; // 00000000 Set continuous mode
    Wire.write(MAGN_ADDRESS_W, data, 2);
    this->wait_ms(7);

    this->Read_Raw_Magn(mag); //first data is thrown away
    this->wait_ms(7);
}

void GY80::Compass(float* comp){
    this->find_roll_pitch_yaw();
    for(int axis = 0; axis<3; axis++){
        comp[axis] = this->compass[axis];
    }  
}

void GY80::Sampling(){
    //magnetometer-accelerometer contribution
    this->Read_Magn(this->north1);
    this->Read_Accel(this->up1);

    //gyroscope contribution
    this->Read_Gyro(this->omega);
    this->v_gyro_update(this->north, this->north2);
    this->v_gyro_update(this->up, this->up2);
    
    for(int axis = 0; axis<3; axis++){
        //weighted average
        this->north[axis] = (1.0f - GYRO_WEIGHT) * this->north1[axis] + GYRO_WEIGHT * this->north2[axis];
        this->up[axis] = (1.0f - GYRO_WEIGHT) * this->up1[axis] + GYRO_WEIGHT * this->up2[axis];

        //normalisasi vektor
        this->v_unit(this->north,this->north);
        this->v_unit(this->up,this->up);
    }
}

void GY80::Compass_No_Gyro(float* comp){
    this->Read_Magn(this->north);
    this->v_unit(this->north, this->north);
    this->Read_Accel(this->up);
    this->v_unit(this->up, this->up);
    this->find_roll_pitch_yaw();
    for(int axis = 0; axis<3; axis++){
        comp[axis] = this->compass[axis];
    }    
}

float GY80::Compass_No_Tilt(){
    float f_mag[3];
    this->Read_Magn(f_mag);
    float Zangle = - atan2(f_mag[1], f_mag[0]);
    if(Zangle<0) Zangle+=2.0f * M_PI;
    Zangle*= RAD2DEG;
    return Zangle;
}

void GY80::Read_Accel(float* accel_v)
{
    this->Read_Raw_Accel(this->accel);

    accel_v[0] = ((short)this->accel[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
    accel_v[1] = ((short)this->accel[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
    accel_v[2] = ((short)this->accel[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;
}

void GY80::Read_Gyro(float* gyro_v)
{
    this->Read_Raw_Gyro(this->gyro);
    
    gyro_v[0] = ((short)this->gyro[0] - GYRO_X_OFFSET) * GYRO_GAIN_X * DEG2RAD; 
    gyro_v[1] = ((short)this->gyro[1] - GYRO_Y_OFFSET) * GYRO_GAIN_Y * DEG2RAD;
    gyro_v[2] = ((short)this->gyro[2] - GYRO_Z_OFFSET) * GYRO_GAIN_Z * DEG2RAD;
   
}

void GY80::Read_Magn(float* magn_v)
{
    this->Read_Raw_Magn(this->mag);

    magn_v[0] = ((short)this->mag[0] - MAGN_X_OFFSET) * MAGN_X_SCALE;
    magn_v[1] = ((short)this->mag[1] - MAGN_Y_OFFSET) * MAGN_Y_SCALE;
    magn_v[2] = ((short)this->mag[2] - MAGN_Z_OFFSET) * MAGN_Z_SCALE;
}

void GY80::Read_Raw_Accel(int16_t* accel_v)
{
    byte buff[6];
    buff[0] = 0x32; // Send address to read from
    Wire.write(ACCEL_ADDRESS, buff, 1);

    if (Wire.read(ACCEL_ADDRESS, buff,6) == 0)  // All bytes received?
    {
        accel_v[0] = ((int16_t) buff[1] << 8 | buff[0]);
        accel_v[1] = ((int16_t) buff[3] << 8 | buff[2]);
        accel_v[2] = ((int16_t) buff[5] << 8 | buff[4]);
    }    
}

void GY80::Read_Raw_Gyro(int16_t* gyro_v)
{
    byte buff[6];

    buff[0] = 0xA8; // 0x28 | (1 << 7) Send address to read from 
    Wire.write(GYRO_ADDRESS, buff, 1);
    // Request 6 bytes
    if (Wire.read(GYRO_ADDRESS, buff,6) == 0)  // All bytes received?
    {
        gyro_v[0] = ((int16_t) buff[1] << 8 | buff[0]);
        gyro_v[1] = ((int16_t) buff[3] << 8 | buff[2]);
        gyro_v[2] = ((int16_t) buff[5] << 8 | buff[4]);
    }
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
    this->b_verbose = debugDisplay;
}

void GY80::Cal_Magn()
{
    byte data[2];

    //positive self-test
    if(!this->b_verbose) printf("callibrating max value..\n");
    data[0] = 0x00;
    data[1] = 0x71; //8 average, 15 Hz, positive self-test
    Wire.write(MAGN_ADDRESS_W, data, 2);
    this->wait_ms(1);

    data[0] = 0x01;
    data[1] = 0x40; // gain=2 (820LSb/Ga)
    Wire.write(MAGN_ADDRESS_W, data, 2);
    this->wait_ms(1);
    
    data[0] = 0x02;
    data[1] = 0x00; // 00000000 Set continuous mode
    Wire.write(MAGN_ADDRESS_W, data, 2);
    this->wait_ms(7);

    this->Read_Raw_Magn(this->mag); //first data is thrown away
    this->wait_ms(7);
    float MaxMag[3] = {0.0f,0.0f,0.0f};
    for(int i=0; i<100; i++){
        this->Read_Raw_Magn(this->mag);
        for(int j=0; j<3; j++){
            MaxMag[j]=(short)this->mag[j] + MaxMag[j];
        }
        wait_ms(7);
    }
    for(int j=0; j<3; j++){
        MaxMag[j]=MaxMag[j]/100.0f;
    }

    //negative self-test
    if(!this->b_verbose) printf("callibrating min value..\n");
    data[0] = 0x00;
    data[1] = 0x72; //8 average, 15 Hz, negative self test (72)
    Wire.write(MAGN_ADDRESS_W, data, 2);
    this->wait_ms(1);

    data[0] = 0x01;
    data[1] = 0x40; // gain=2 (820LSb/Ga)
    Wire.write(MAGN_ADDRESS_W, data, 2);
    this->wait_ms(1);
    
    data[0] = 0x02;
    data[1] = 0x00; // 00000000 Set continuous mode
    Wire.write(MAGN_ADDRESS_W, data, 2);
    this->wait_ms(7);

    this->Read_Raw_Magn(this->mag); //first data is thrown away
    this->wait_ms(7);
    float MinMag[3] = {0.0f,0.0f,0.0f};
    for(int i=0; i<100; i++){
        this->Read_Raw_Magn(this->mag);
        for(int j=0; j<3; j++){
            MinMag[j]=(short)this->mag[j] + MinMag[j];
        }
        wait_ms(7);
    }
    for(int j=0; j<3; j++){
        MinMag[j]=MinMag[j]/100.0f;
    }

    //display
    if(!this->b_verbose){
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
    this->Magn_Init();
}

void GY80::wait_ms(int t)
{
    wait_us(1000*t);
}

void GY80::v_cross_p(float* a, float* b, float* product){
    for(int axis = 0; axis<3; axis++){
        product[axis] = a[(axis+1)%3] * b[(axis+2)%3] - a[(axis+2)%3] * b[(axis+1)%3];
    }
}

float GY80::v_length(float* vector){
    float v_length_square = 0.0f;
    for(int axis = 0; axis<3; axis++){
        v_length_square += vector[axis] * vector[axis];
    }
    return sqrt(v_length_square);
}

void GY80::v_unit(float* vector, float* u_vector){
    float length = this->v_length(vector);
    for(int axis = 0; axis<3; axis++){
        u_vector[axis] = vector[axis] / length;
    }
}

void GY80::v_gyro_update(float* prev_v, float* curr_v){
    //find previous angle
    float theta[3];
    theta[0] = atan2(prev_v[2],prev_v[1]);
    theta[1] = atan2(prev_v[0], prev_v[2]);
    theta[2] = atan2(prev_v[1], prev_v[0]);

    //find current angle
    for(int axis = 0; axis<3; axis++){
        theta[axis] -= this->omega[axis] * this->timeSampling / 1000000.0f;
    }

    //find curr_v
    float sThetaY = sin(theta[1]); float tThetaZ = tan(theta[2]);
    curr_v[0] = sThetaY / (sqrt(1 + tThetaZ * tThetaZ * sThetaY * sThetaY));
    float sThetaZ = sin(theta[2]); float tThetaX = tan(theta[0]);
    curr_v[1] = sThetaZ / (sqrt(1 + tThetaX * tThetaX * sThetaZ * sThetaZ));
    curr_v[2] = sqrt(1 - curr_v[0] * curr_v[0] - curr_v[1] * curr_v[1]);
    if(sin(theta[0]) < 0.0f) curr_v[2] = - curr_v[2];
}

void GY80::find_roll_pitch_yaw(){
    //find world unit vector in respect to b coordinate
    float xw[3], yw[3], zw[3];
    for(int axis = 0; axis<3; axis++){ //zw = up
        zw[axis] = this->up[axis];
    }
    this->v_cross_p(this->up, this->north, yw);//find yw
    this->v_cross_p(yw, zw, xw); //find xw
    
    //find roll pitch yaw
    float pitch = asin(- zw[0]);
    float cpitch = cos(pitch);
    float yaw = asin(zw[1] / cpitch);
    if((cpitch * cos(yaw) / zw[2]) < 0.0f){
        pitch = M_PI - pitch;
    }
    if(pitch > M_PI) pitch -= 2.0f * M_PI;
    cpitch = cos(pitch);
    float roll = asin(yw[0] / cpitch);
    if((cos(roll) * cpitch / xw[0]) < 0.0f){
        roll = M_PI - roll;
    }
    if(roll < 0.0f) roll += 2.0f * M_PI;
    
    //output
    this->compass[0] = roll * RAD2DEG;
    this->compass[1] = pitch * RAD2DEG;
    this->compass[2] = yaw * RAD2DEG;
}