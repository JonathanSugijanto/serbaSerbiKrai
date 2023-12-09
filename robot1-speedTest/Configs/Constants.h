#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

/* TIME SAMPLINGS */
#define SAMP_BASE_MOTOR_US 5173
#define SAMP_PID_BASE_MOTOR_US 4173
#define SAMP_BASE_MOTOR_ENCODER_US 5000

#define SAMP_IK_US 20000
#define SAMP_UPD_POS_US 12000
#define SAMP_BASE_ODOMETRY_US 12731

#define SAMP_OTOM_US 500000
#define SAMP_PARALLEL_PARK_US 500000

#define SAMP_STICK_US 13000
#define SAMP_STICK_MS 13

#define BASE_FL_KP 0.0f
#define BASE_FL_KI 0.0f
#define BASE_FL_KD 0.0f
#define SAMP_PID_BASE_MOTOR_MS (SAMP_PID_BASE_MOTOR_US/1000.0)

#define BASE_FR_KP 0.0f
#define BASE_FR_KI 0.0f
#define BASE_FR_KD 0.0f
#define SAMP_PID_BASE_MOTOR_MS (SAMP_PID_BASE_MOTOR_US/1000.0)

#define BASE_BL_KP 0.0f
#define BASE_BL_KI 0.0f
#define BASE_BL_KD 0.0f
#define SAMP_PID_BASE_MOTOR_MS (SAMP_PID_BASE_MOTOR_US/1000.0)

#define BASE_BR_KP 0.0f
#define BASE_BR_KI 0.0f
#define BASE_BR_KD 0.0f
#define SAMP_PID_BASE_MOTOR_MS (SAMP_PID_BASE_MOTOR_US/1000.0)

#define BASE_MID_KP 0.0f
#define BASE_MID_KI 0.0f
#define BASE_MID_KD 0.0f
#define SAMP_PID_BASE_MOTOR_MS (SAMP_PID_BASE_MOTOR_US/1000.0)


/* BASE MOTOR CONTROL */
// ga dipake tp jg dibutuhin bwt ControlMotor
#define BASE_MOTOR_V_LIM 1.0f

/* BASE MOTOR VARIABLE */
#define PPR_FL 384.0f
#define PPR_FR 384.0f
#define PPR_BL 538.2f
#define PPR_BR 537.2f
#define PPR_MID 538.0f

#define TRANSLATION_BASE_SPEED 0.45f
#define ROTATION_BASE_SPEED (1.05 * PI / 6)
#define TRANSLATION_BOOST_MULTIPLIER 2.15f
#define ROTATION_BOOST_MULTIPLIER 1.8f
#define TRANSLATION_HINDER_MULTIPLIER 0.4f
#define ROTATION_HINDER_MULTIPLIER 0.4f

/* BASE MOTOR INPUT LIMIT */
#define MAX_FL_INPUT_LIMIT 3.3 // 2.6
#define MAX_FR_INPUT_LIMIT 3.3 // 2.7
#define MAX_BR_INPUT_LIMIT 3.3 // 2.2
#define MAX_BL_INPUT_LIMIT 3.3 // 2.8

/* CONTROL4OMNI VARIABLES */
/* Kalau mau ubah, perlu ubah sampai library-nya, jangan diapa-apain */
#ifndef PI
#define PI 3.1415926535897932384626433832795f
#endif

#define ERROR_THRESHOLD 5

#define ENC_MOTOR_SAMP_US_DEF SAMP_BASE_MOTOR_ENCODER_US

#define WHEEL_RAD 0.075

#define S_TO_US 1000000
#define MS_TO_US 1000

#define MAX_ACCEL_Y 8
#define MAX_ACCEL_X 8
#define MAX_ACCEL_W (20 * PI)

#define R_BASE 0.3341295
// #define R_BASE 0.450

//shooter constants
#define PPR_shooter 103.3f
#define PPR_reloader 104.4f
#define PPR_angle 538.0f

#endif
