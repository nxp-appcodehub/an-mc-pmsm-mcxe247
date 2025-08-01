/*
    * Copyright 2025 NXP
    *
    * SPDX-License-Identifier: BSD-3-Clause
*/

/*
    * FILE NAME: ../../source/m2_pmsm_appconfig.h
    * DATE: Wed Jul 09 2025, 16:45:39
*/

/*
{
    "parameters": {
        "parametersPP": 2,
        "parametersRs": 0.45,
        "parametersLd": 0.000375,
        "parametersLq": 0.000429,
        "parametersKe": 0.0138,
        "parametersJ": 0.0000016,
        "parametersIphNom": 2,
        "parametersUphNom": 15,
        "parametersNnom": 4000,
        "parametersImax": 8.25,
        "parametersUdcbMax": 60.8,
        "parametersUdcbTrip": 29,
        "parametersUdcbUnder": 16,
        "parametersUdcbOver": 32,
        "parametersNover": 4399,
        "parametersNmin": 300,
        "parametersEblock": 1.4,
        "parametersEblockPer": 2000,
        "parametersNmax": 4400,
        "parametersUdcbIIRf0": 100,
        "parametersCalibDuration": 0.2,
        "parametersFaultDuration": 2,
        "parametersFreewheelDuration": 1.5,
        "parametersScalarUqMin": 1,
        "parametersAlignVoltage": 1.2,
        "parametersAlignDuration": 0.4,
        "parametersScalarVHzRatio": 100
    },
    "currentLoop": {
        "currentLoopSampleTime": 0.0001,
        "currentLoopF0": 288,
        "currentLoopKsi": 1,
        "currentLoopOutputLimit": 90
    },
    "speedLoop": {
        "speedLoopSampleTime": 0.001,
        "speedLoopF0": 5,
        "speedLoopKsi": 1,
        "speedLoopIncUp": 10000,
        "speedLoopIncDown": 10000,
        "speedLoopCutOffFreq": 100,
        "speedLoopUpperLimit": 2,
        "speedLoopLowerLimit": -2,
        "speedLoopSLKp": 0.003,
        "speedLoopSLKi": 0.09,
        "speedLoopManualConstantTunning": false
    },
    "positionLoop": {
        "positionLoopSampleTime": 0.001,
        "positionLoopF0": "",
        "positionLoopKsi": "",
        "servo_positionLoopUpperLimit": "",
        "servo_positionLoopLowerLimit": "",
        "servo_speedLoopUpperLimit": "",
        "servo_speedLoopLowerLimit": ""
    },
    "sensorless": {
        "sensorlessBemfObsrvF0": 300,
        "sensorlessBemfObsrvKsi": 1,
        "sensorlessTrackObsrvF0": 70,
        "sensorlessTrackObsrvKsi": 1,
        "sensorlessTrackObsrvIIRSpeedCutOff": 400,
        "sensorlessStartupRamp": 5000,
        "sensorlessStartupCurrent": 0.35,
        "sensorlessMergingSpeed": 300,
        "sensorlessMergingCoeff": 100
    }
}
*/

/*
{
    "motorName": "Linix",
    "motorDescription": "Configuration for Linix motor."
}
*/

#ifndef __M1_PMSM_APPCONFIG_H 
#define __M1_PMSM_APPCONFIG_H 

/* PARAMETERS*/
#define M1_MOTOR_PP (2)
#define M1_I_PH_NOM (2.0F)
#define M1_N_NOM (837.758F)
#define M1_I_MAX (8.25F)
#define M1_U_DCB_MAX (60.8F)
#define M1_U_DCB_TRIP (29.0F)
#define M1_U_DCB_UNDERVOLTAGE (16.0F)
#define M1_U_DCB_OVERVOLTAGE (32.0F)
#define M1_N_OVERSPEED (921.324F)
#define M1_N_MIN (62.8319F)
#define M1_E_BLOCK_TRH (1.4F)
#define M1_E_BLOCK_PER (2000)
#define M1_N_MAX (921.534F)
#define M1_CALIB_DURATION (200)
#define M1_FAULT_DURATION (2000)
#define M1_FREEWHEEL_DURATION (1500)
#define M1_SCALAR_UQ_MIN (1.0F)
#define M1_ALIGN_VOLTAGE (1.2F)
#define M1_ALIGN_DURATION (4000)
#define M1_U_MAX (35.1029F)
#define M1_FREQ_MAX (146.667F)
#define M1_N_ANGULAR_MAX (4.77465F)
#define M1_UDCB_IIR_B0 (0.0304590F)
#define M1_UDCB_IIR_B1 (0.0304590F)
#define M1_UDCB_IIR_A1 (0.939082F)
#define M1_SCALAR_VHZ_FACTOR_GAIN (0.112500F)
#define M1_SCALAR_INTEG_GAIN ACC32(0.0293333)
#define M1_SCALAR_RAMP_UP (0.0333333F)
#define M1_SCALAR_RAMP_DOWN (0.0333333F)
/* CURRENTLOOP*/
#define M1_D_KP_GAIN (0.907168F)
#define M1_D_KI_GAIN (0.0613968F)
#define M1_Q_KP_GAIN (1.10260F)
#define M1_Q_KI_GAIN (0.0702380F)
#define M1_Q_IIR_ZC_B0 (0.0598872F)
#define M1_Q_IIR_ZC_B1 (0.0598872F)
#define M1_Q_IIR_ZC_A1 (0.880226F)
#define M1_CLOOP_LIMIT (0.519615F)
/* SPEEDLOOP*/
#define M1_SPEED_RAMP_UP (2.09440F)
#define M1_SPEED_RAMP_DOWN (2.09440F)
#define M1_SPEED_LOOP_HIGH_LIMIT (2.0F)
#define M1_SPEED_LOOP_LOW_LIMIT (-2.0F)
#define M1_SPEED_PI_PROP_GAIN (0.00364243F)
#define M1_SPEED_PI_INTEG_GAIN (0.0000286075F)
#define M1_SPEED_IIR_B0 (0.239057F)
#define M1_SPEED_IIR_B1 (0.239057F)
#define M1_SPEED_IIR_A1 (0.521886F)
#define M1_SPEED_IIR_ZC_B0 (0.00779278F)
#define M1_SPEED_IIR_ZC_B1 (0.00779278F)
#define M1_SPEED_IIR_ZC_A1 (0.984414F)
/* POSITIONLOOP*/
#define M1_SERVO_POSITION_P_HIGH_LIMIT (1256.64F)
#define M1_SERVO_POSITION_P_LOW_LIMIT (-1256.64F)
#define M1_SERVO_POSITION_P_PROP_GAIN (1052.76F)
#define M1_SERVO_FEED_FRWD_K1 (16.7552F)
#define M1_SERVO_FEED_FRWD_K2 (0.0666667F)
#define M1_SERVO_IIR_ZC_B0 (0.0F)
#define M1_SERVO_IIR_ZC_B1 (0.0F)
#define M1_SERVO_IIR_ZC_A1 (0.881765F)
#define M1_SERVO_SPEED_PI_PROP_GAIN (0.0172065F)
#define M1_SERVO_SPEED_PI_INTEG_GAIN (0.00108111F)
#define M1_SERVO_SPEED_PI_HIGH_LIMIT 
#define M1_SERVO_SPEED_PI_LOW_LIMIT 
/* SENSORLESS*/
#define M1_OL_START_RAMP_INC (0.104720F)
#define M1_OL_START_I (0.35F)
#define M1_MERG_SPEED_TRH (62.8319F)
#define M1_MERG_COEFF FRAC16(0.00100708)
#define M1_I_SCALE (0.892857F)
#define M1_U_SCALE (0.238095F)
#define M1_E_SCALE (0.238095F)
#define M1_WI_SCALE (0.000102143F)
#define M1_BEMF_DQ_KP_GAIN (0.963717F)
#define M1_BEMF_DQ_KI_GAIN (0.133240F)
#define M1_TO_KP_GAIN (879.646F)
#define M1_TO_KI_GAIN (19.3444F)
#define M1_TO_THETA_GAIN (0.0000318310F)
#define M1_TO_SPEED_IIR_B0 (0.111635F)
#define M1_TO_SPEED_IIR_B1 (0.111635F)
#define M1_TO_SPEED_IIR_A1 (0.776730F)
/* USER INPUT START */
#define AA 1
#define BB 3
#define CC 4
#define DD 2
/* USER INPUT END */
#endif /* __M1_PMSM_APPCONFIG_H */
