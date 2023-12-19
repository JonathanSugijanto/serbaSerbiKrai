/******************************************************************************
*                           GY80 
*               library oleh Jonathan Sugijanto
*   GY80 adalah modul IMU dengan 10 DOF yang literally adalah gabungan 4 IC:
*
*   HMC5883L (3-Axis Digital Compass), I2C Address 0x1E
*       datasheet: https://cdn-shop.adafruit.com/datasheets/HMC5883L_3-Axis_Digital_Compass_IC.pdf 
*       for reference: magnetic field on Bandung should be:
*           Magnetic Declination: +0° 33'
*           Declination is POSITIVE (EAST)
*           Inclination: 30° 30'
*           Magnetic field strength: 44794.0 nT or 0.448 Ga
*   ADXL345 (3-Axis Digital Accelerometer), I2C Address 0×53
*       datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/adxl345.pdf 
*   L3G4200D (3-Axis Angular Rate Sensor), I2C Address 0×69
*       datasheet: https://www.iot-lab.info/assets/misc/docs/iot-lab-m3/L3G4200D.pdf  
*   BMP085 (Barometric Pressure / Temperature Sensor), I2C Address 0×77
*
*   technical spec:
*   Supply Voltage: 3V – 5V
*   Communication Mode: I2C
*   Dimensions: 2.5 cm x 1.7 cm x 0.2 cm
*   Weight: 31 g
*
*   library spec:
*   update rate: 75 Hz max (magnetometer max update rate)
*   minimal efective timeSampling: 1250 us (gyro max update rate: 800 Hz)
*   Calibrated for 5V supply only
*   
*   library ini dibuat untuk aplikasi tilt-compensated compass (pengganti CMPS12) dengan compass mengarah ke sumbu-X module
*******************************************************************************/

#ifndef _GY80_H_
#define _GY80_H_

//pick either of the two modes:
#define SCALED_MAGN_OUTPUT //using min max data from "data magnet full.csv" //output a scaled data where 100 is the ambient magnetic field
// #define CALLIBRATED_MAGN_OUTPUT //using average data from "data callibrate moving.csv" //output in uT (note: 1 Gauss = 100uT)

#include "mbed.h"
#define I2C_FREQ 400000
#define SCL PB_8
#define SDA PB_9

#define GYRO_WEIGHT 0.8f //gyro to magnetometer-accelerometer weighted average ratio
#define AVG_COUNT 5

#define ACCEL_ADDRESS (0xA6) // 0x53 = 0xA6 / 2
#define MAGN_ADDRESS_W  (0x3C) // 0x1E = 0x3C / 2
#define MAGN_ADDRESS_R  (0x3D)
#define GYRO_ADDRESS  (0xD2) // 0x69 = 0xD2 / 2

// SENSOR CALIBRATION (xyz offset and scaling)
/*****************************************************************/
// How to calibrate: rotate the module slowly in every possible angle
// Put MIN/MAX of the raw value read here
// Accelerometer
// "accel x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define ACCEL_X_MIN (-249.0f)
#define ACCEL_X_MAX (268.0f)
#define ACCEL_Y_MIN (-253.0f)
#define ACCEL_Y_MAX (268.0f)
#define ACCEL_Z_MIN (-251.0f)
#define ACCEL_Z_MAX (265.0f)

// Magnetometer
// "magn x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#ifdef SCALED_MAGN_OUTPUT
#define MAGN_X_MIN (-575.0f) //datasheet: -479.5
#define MAGN_X_MAX (498.0f) //datasheet: 479.5
#define MAGN_Y_MIN (-662.0f) //datasheet: -479.5
#define MAGN_Y_MAX (255.0f) //datasheet: 479.5
#define MAGN_Z_MIN (-605.0f) //datasheet: -479.5
#define MAGN_Z_MAX (287.0f) //datasheet: 479.5
#endif

#ifdef CALLIBRATED_MAGN_OUTPUT
// please use the cal_magn function to find this
//last update: due to poor callibrated performance, I dont use this
#define MAGN_X_MIN (-1411.186149f) //datasheet: -1589.2 //moving self-test: -1411.186149
#define MAGN_X_MAX (1527.426455f) //datasheet: 1589.2 //moving self-test: 1527.426455
#define MAGN_Y_MIN (-1337.351606f) //datasheet: -1589.2 //moving self-test: -1337.351606
#define MAGN_Y_MAX (1439.017619f) //datasheet: 1589.2 //moving self-test: 1439.017619
#define MAGN_Z_MIN (-1319.683693f) //datasheet: -1479.6 //moving self-test: -1319.683693
#define MAGN_Z_MAX (1480.912522f) //datasheet: 1479.6 //moving self-test: 1480.912522
#endif

// Gyroscope
// "gyro x,y,z (current/average) = .../OFFSET_X  .../OFFSET_Y  .../OFFSET_Z
#define GYRO_X_OFFSET (-27.39333333f)
#define GYRO_Y_OFFSET (34.6f)
#define GYRO_Z_OFFSET (4.303333333f)

//*****************************************************************************/

#define GRAVITY 255.0f  //this equivalent to 1G in the raw data coming from the accelerometer 
#define G_TO_CMpS2 980.665f

#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_SCALE (G_TO_CMpS2 * GRAVITY / 255.0f / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (G_TO_CMpS2 * GRAVITY / 255.0f / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (G_TO_CMpS2 * GRAVITY / 255.0f / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))

//base 
#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)

#ifdef SCALED_MAGN_OUTPUT
#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET)) 
#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET)) 
#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET)) 
#endif

#ifdef CALLIBRATED_MAGN_OUTPUT
#define MAGN_X_SCALE (116.0f / (MAGN_X_MAX - MAGN_X_OFFSET)) //use 116.0f for output in uT //use 100 for scalled
#define MAGN_Y_SCALE (116.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET)) //use 116.0f for output in uT 
#define MAGN_Z_SCALE (108.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET)) //use 108.0f for output in uT 
#endif

// Gyro gain (conversion from raw to degree per seconds)
#define GYRO_GAIN 0.07f //based on datasheet, or 0.061035156f based on max min range
#define GYRO_GAIN_X 0.07f //X axis Gyro gain
#define GYRO_GAIN_Y 0.07f //Y axis Gyro gain
#define GYRO_GAIN_Z 0.175f //Z axis Gyro gain (0.175?)

#define DEG2RAD 0.01745329252f  // *pi/180
#define RAD2DEG 57.2957795131f  // *180/pi
#ifndef M_PI
#define M_PI 3.14159265359f
#endif
#define ERROR_TOL (20.0f * DEG2RAD)

typedef char byte;

class GY80
{
public:
    I2C Wire;

    /** Create GY80 inteface
     * @param sda_pin mbed pin to use for I2C SDA
     * @param scl_pin mbed pin to use for I2C SCL
     * @param timeSampling: in us, 10000 is good (2 deg/min gyro drift)
     */
    GY80(int timeSampling);
    
    /** Create GY80 inteface
     * @param sda_pin mbed pin to use for I2C SDA
     * @param scl_pin mbed pin to use for I2C SCL
     * @param timeSampling: in us, 10000 is good (2 deg/min gyro drift)
     */
    GY80(PinName sda_pin, PinName scl_pin, int timeSampling);

    ~GY80();

    /**Get relative Angle of module's X axis about the vertical axis of the world
     * NOTE: if you want to get two of the angle, pitch, and yaw at once, it is reccomended to use Compass_rel() function instead
     * @return relative angle in degrees from 0.0f to 360.0f
     * tilt compensation uses Compass(float* comp) function
     * @details simple combination of magnetometer and accelerometer
    */
    float get_Angle();
    //set current relative Angle
    void set_Angle(float new_angle); 

    /**Get Pitch of module's X axis about the horizontal axis of the world
     * @return Pitch in degrees from -180.0f to 180.0f
     * tilt compensation uses Compass(float* comp) function
     * @details simple combination of magnetometer and accelerometer
    */
    float get_Pitch();

    /**Get Roll about the X axis of the module
     * @return angle in degrees from 0.0f to 360.0f
     * tilt compensation uses Compass(float* comp) function
     * @details simple combination of magnetometer and accelerometer
    */
    float get_Rotation();

    /**Assume acceleration is always vertical down
     * Combining compass and accelerometer data (with relative zero roll angle)
     * only effective in the range (degrees): roll(0 - 360), pitch(-180 - 180), yaw(-90 - 90)
     * @param comp address of float array[3] for data roll (Z angle), pitch (inclination), yaw (rotation about the module's X)
     * @details penurunan: https://drive.google.com/drive/folders/1PRHnzby6aZBDhad5YUobmHNFptBi6PaO?usp=sharing 
    */
    void Compass_rel(float* comp);

    /**Compass function version 2
     * Simple weighted average of gyroscope and magnetometer-accelerometer
     * Note: this function needs sampling!
     * only effective in the range (degrees): roll(0 - 360), pitch(-180 - 180), yaw(-90 - 90)
     * @param comp address of float array[3] for data roll (Z angle), pitch (inclination), yaw (rotation about the module's X)
     * @details penurunan: https://drive.google.com/drive/folders/1PRHnzby6aZBDhad5YUobmHNFptBi6PaO?usp=sharing 
    */
    void Compass(float* comp);

    void Sampling();

    void ex_Sampling();
    void ex_Compass(float* speed, float* comp);

    void ex1_Sampling();
    void ex1_Compass(float* comp);

    /**Compass function version 1
     * Assume acceleration is always vertical down
     * Combining compass and accelerometer data, doesn't need sampling
     * only effective in the range (degrees): roll(0 - 360), pitch(-180 - 180), yaw(-90 - 90)
     * @param address of float array[3] for data roll (Z angle), pitch (inclination), yaw (rotation about the module's X)
     * @details penurunan: https://drive.google.com/drive/folders/1PRHnzby6aZBDhad5YUobmHNFptBi6PaO?usp=sharing 
    */
    void Compass_No_Gyro(float* comp);

    /**Compass function version 0
     * Assume Z is always vertical up
     * Using just the compass data, doesn't need sampling
     * @return Z_angle from where magnetic north is
    */
    float Compass_No_Tilt();

    void Read_Accel(float* accel_v);
    void Read_Gyro(float* gyro_v);

    /**Read scalled magnetic field vector (100 for ambient magnetic field)
     * @param address of float array[3] for data x,y,z
     * @details based on "data magnet full.csv"
     */
    void Read_Magn(float* magn_v);

    void Read_Raw_Accel(int16_t* accel_v);
    void Read_Raw_Gyro(int16_t* gyro_v);

    /**Read raw magnetic field vector
     * output in 1370 LSb/Ga uncallibrated
     * @param address of int16_t array[3] for data x,y,z
    */
    void Read_Raw_Magn(int16_t* magn_v);

    //turn on debug
    //for developer purposes
    void set_b_verbose(bool debugDisplay);

    /**Callibrating the offset and device scalling (averaged 100 x 8 times)
     * ONLY USE THIS AT TEMP 25C
     * the callibration takes about 1.4s and should be done at rest/slowly moving, away from any magnetic field source
     * the output is in the form of printf,so the minmax value need to be changed manually in the definition
     */
    void Cal_Magn();
    

private:
    int16_t accel[3];
    int16_t gyro[3];
    int16_t mag[3];
    float compass[3];

    float omega[3];
    float up1[3], north1[3]; //unit vector of up and north in respect to module using accelerometer-magnetometer
    float up2[3], north2[3]; //unit vector of up and north in respect to module using gyroscope
    float up[3], north[3]; //unit vector of up and north in respect to module
    uint32_t up_timer[3], north_timer[3];
    float compassBuffer[AVG_COUNT][3];
    uint32_t count = 0;

    void Accel_Init();
    void Gyro_Init();
    void Magn_Init();
    
    float zero_angle_rel = 0.0f;
    bool b_verbose = false;
    int timeSampling = 10000;

    void wait_ms(int t);
    void v_cross_p(float* a, float* b, float* product); //return the cross product of a[3] and b[3]
    float v_length(float* vector);
    void v_unit(float* vector, float* u_vector);
    void v_gyro_update(float* prev_v, float* curr_v, uint32_t* timer);
    void find_roll_pitch_yaw(float* v_north, float* v_up, float* v_compass);
};

#endif