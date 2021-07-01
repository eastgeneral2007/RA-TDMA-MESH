/**
 * navdata.h
 */

#ifndef _NAVDATA_H_
#define _NAVDATA_H_

#include <stdbool.h>
#include <inttypes.h>
/*******************************************************************************
            NAVDATA OPTION OFFSETS
*******************************************************************************/

/* --- General --- */
#define HEADER_OFFSET                       (16)
#define TAG_OFFSET                          (0)
#define SIZE_OFFSET                         (2)


/* --- navdata_t --- */
#define NAV_HEADER_STATE                    (4)
#define NAV_HEADER_SEQUENCE                 (8)


/* --- navdata_demo_t --- */
/* Minimum data needed */
#define NAV_DEMO_TAG                        (0)
#define NAV_DEMO_SIZE                       (148)

#define NAV_DEMO_BATTERY                    (8)
#define NAV_DEMO_PITCH                      (12)
#define NAV_DEMO_ROLL                       (16)
#define NAV_DEMO_YAW                        (20)
#define NAV_DEMO_ALTITUDE                   (24)

#define NAV_DEMO_VX                         (28)
#define NAV_DEMO_VY                         (32)
#define NAV_DEMO_VZ                         (36)


/* --- navdata_time_t --- */
/* The AR Drone current time */
#define NAV_TIME_TAG                        (1)
#define NAV_TIME_SIZE                       (8)
#define NAV_TIME_TIME                       (4)


/* --- navdata_raw_measures_t --- */
/* Raw measurements coming from the PIC */
#define NAV_RAW_MEAS_TAG                    (2)
#define NAV_RAW_MEAS_SIZE                   (40)

#define NAV_RAW_MEAS_ACC_X                  (4)
#define NAV_RAW_MEAS_ACC_Y                  (6)
#define NAV_RAW_MEAS_ACC_Z                  (8)

#define NAV_RAW_MEAS_GYRO_X                 (10)
#define NAV_RAW_MEAS_GYRO_Y                 (12)
#define NAV_RAW_MEAS_GYRO_Z                 (14)

#define NAV_RAW_MEAS_GYRO_110_1             (16)
#define NAV_RAW_MEAS_GYRO_110_2             (18)

#define NAV_RAW_MEAS_VBAT                   (20)

#define NAV_RAW_MEAS_US_DEBUT               (24)
#define NAV_RAW_MEAS_US_FIN                 (26)
#define NAV_RAW_MEAS_US_ASSOCIATION         (28)
#define NAV_RAW_MEAS_US_COURBE_TEMPS        (30)
#define NAV_RAW_MEAS_US_COURBE_VALEUR       (32)
#define NAV_RAW_MEAS_US_COURBE_REF          (34)
#define NAV_RAW_MEAS_FLAG_ECHO_INI          (36)


/* --- navdata_phys_measures_t --- */
/* Filtered values */
#define NAV_PHYS_MEAS_TAG                   (3)
#define NAV_PHYS_MEAS_SIZE                  (46)

#define NAV_PHYS_MEAS_ACCS_TEMP             (4)
#define NAV_PHYS_MEAS_GYRO_TEMP             (8)

#define NAV_PHYS_MEAS_ACC_X                 (10)
#define NAV_PHYS_MEAS_ACC_Y                 (14)
#define NAV_PHYS_MEAS_ACC_Z                 (18)

#define NAV_PHYS_MEAS_GYRO_X                (22)
#define NAV_PHYS_MEAS_GYRO_Y                (26)
#define NAV_PHYS_MEAS_GYRO_Z                (30)

#define NAV_PHYS_MEAS_ALIM_3V3              (34)
#define NAV_PHYS_MEAS_VREF_EPSON            (38)
#define NAV_PHYS_MEAS_VREF_IDG              (42)


/* --- navdata_gyros_offsets_t --- */
/* Gyro offsets */
#define NAV_GYRO_OFFSET_TAG                 (4)
#define NAV_GYRO_OFFSET_SIZE                (16)

#define NAV_GYRO_OFFSET_X                   (4)
#define NAV_GYRO_OFFSET_Y                   (8)
#define NAV_GYRO_OFFSET_Z                   (12)


/* --- navdata_euler_angles_t */
/* Fused euler angles */
#define NAV_EULER_TAG                       (5)
#define NAV_EULER_SIZE                      (12)

#define NAV_EULER_THETA                     (4)
#define NAV_EULER_PHI                       (5)


/* --- navdata_references_t */
#define NAV_REFS_TAG                        (6)
#define NAV_REFS_SIZE                       (36)

#define NAV_REFS_THETA                      (4)
#define NAV_REFS_PHI                        (8)
#define NAV_REFS_THETA_I                    (12)
#define NAV_REFS_PHI_I                      (16)
#define NAV_REFS_PITCH                      (20)
#define NAV_REFS_ROLL                       (24)
#define NAV_REFS_YAW                        (28)
#define NAV_REFS_PSI                        (32)


/* --- navdata_trims_t --- */
#define NAV_TRIMS_TAG                       (7)
#define NAV_TRIMS_SIZE                      (16)

#define NAV_TRIMS_ANGULAR_RATES             (4)
#define NAV_TRIMS_THETA                     (8)
#define NAV_TRIMS_PHI                       (12)


/* --- navdata_rc_references */
#define NAV_RC_TAG                          (8)
#define NAV_RC_SIZE                         (24)

#define NAV_RC_PITCH                        (4)
#define NAV_RC_ROLL                         (8)
#define NAV_RC_YAW                          (4)
#define NAV_RC_GAZ                          (4)
#define NAV_RC_AG                           (4)


/* --- navdata_pwm_t --- */
/* Data used to control the motors */
#define NAV_PWM_TAG                         (9)
#define NAV_PWM_SIZE                        (68)

#define NAV_PWM_MOTOR1                      (4)
#define NAV_PWM_MOTOR2                      (5)
#define NAV_PWM_MOTOR3                      (6)
#define NAV_PWM_MOTOR4                      (7)

#define NAV_PWM_SAT_MOTOR1                  (8)
#define NAV_PWM_SAT_MOTOR2                  (9)
#define NAV_PWM_SAT_MOTOR3                  (10)
#define NAV_PWM_SAT_MOTOR4                  (11)

#define NAV_PWM_GAZ_FEED_FORWARD            (12)
#define NAV_PWM_GAZ_ALTITUDE                (16)
#define NAV_PWM_ALTITUDE_INTEGRAL           (20)
#define NAV_PWM_VZ_REF                      (24)
#define NAV_PWM_U_PITCH                     (28)
#define NAV_PWM_U_ROLL                      (32)
#define NAV_PWM_U_YAW                       (36)
#define NAV_PWM_YAW_U_I                     (40)
#define NAV_PWM_U_PITCH_PLANIF              (44)
#define NAV_PWM_U_ROLL_PLANIF               (48)
#define NAV_PWM_U_YAW_PLANIF                (52)
#define NAV_PWM_U_GAZ_PLANIF                (56)
#define NAV_PWM_CURRENT_MOTOR1              (60)
#define NAV_PWM_CURRENT_MOTOR2              (62)
#define NAV_PWM_CURRENT_MOTOR3              (64)
#define NAV_PWM_CURRENT_MOTOR4              (66)


/* --- navdata_altitude_t --- */
/* Estimated values with a relation to altitude */
#define NAV_ALT_TAG                         (10)
#define NAV_ALT_SIZE                        (56)

#define NAV_ALT_ALT_VISION                  (4)
#define NAV_ALT_ALT_VZ                      (8)
#define NAV_ALT_ALT_REF                     (12)
#define NAV_ALT_ALT_RAW                     (16)
#define NAV_ALT_OBS_ACC_Z                   (20)
#define NAV_ALT_OBS_ALT                     (24)
#define NAV_ALT_OBS_X_1                     (28)                        
#define NAV_ALT_OBS_X_2                     (32)
#define NAV_ALT_OBS_X_3                     (36)
#define NAV_ALT_OBS_STATE                   (40)
#define NAV_ALT_EST_VB_1                    (44)
#define NAV_ALT_EST_VB_2                    (48)
#define NAV_ALT_EST_STATE                   (52)


/* --- navdata_vision_raw_t --- */
/* Vision's estimated velocities */
#define NAV_VIS_RAW_TAG                     (11)
#define NAV_VIS_RAW_SIZE                    (16)

#define NAV_VIS_RAW_TX                      (4)
#define NAV_VIS_RAW_TY                      (8)
#define NAV_VIS_RAW_TZ                      (12)


/* --- navdata_vision_of_t --- */
#define NAV_VIS_OF_TAG                      (12)
#define NAV_VIS_OF_SIZE                     (44)


/* --- navdata_vision_t --- */
/* Data used when computing vision */
#define NAV_VISION_TAG                      (13)
#define NAV_VISION_SIZE                     (92)

#define NAV_VISION_STATE                    (4)
#define NAV_VISION_MISC                     (8)

#define NAV_VISION_PHI_TRIM                 (12)
#define NAV_VISION_PHI_REF_PROP             (16)
#define NAV_VISION_THETA_TRIM               (20)
#define NAV_VISION_THETA_REF_PROP           (24)

#define NAV_VISION_NEW_RAW_PICTURE          (28)
#define NAV_VISION_THETA_CAPTURE            (32)
#define NAV_VISION_PHI_CAPTURE              (36)
#define NAV_VISION_PSI_CAPTURE              (40)
#define NAV_VISION_ALTITUDE_CAPTURE         (44)
#define NAV_VISION_TIME_CAPTURE             (48)

#define NAV_VISION_BODY_V_X                 (52)
#define NAV_VISION_BODY_V_Y                 (56)
#define NAV_VISION_BODY_V_Z                 (60)

#define NAV_VISION_DELTA_PHI                (64)
#define NAV_VISION_DELTA_THETA              (68)
#define NAV_VISION_DELTA_PSI                (72)

#define NAV_VISION_GOLD_DEFINED             (76)
#define NAV_VISION_GOLD_RESET               (80)
#define NAV_VISION_GOLD_X                   (84)
#define NAV_VISION_GOLD_Y                   (88)


/* --- navdata_vision_perf_t --- */
/* Performance data collected when profiling vision code */
#define NAV_VIS_PERF_TAG                    (14)
#define NAV_VIS_PERF_SIZE                   (108)

#define NAV_VIS_PERF_SZO                    (4)
#define NAV_VIS_PERF_CORNERS                (8)
#define NAV_VIS_PERF_OCMPUTE                (12)
#define NAV_VIS_PERF_TRACKING               (16)
#define NAV_VIS_PERF_TRANS                  (20)
#define NAV_VIS_PERF_UPDATE                 (24)

// Ignored an array of twenty timestamps here


/* --- navdata_trackers_send_t --- */
/* Position of all trackers computed by vision */
#define NAV_TRACKERS_TAG                    (15)
#define NAV_TRACKERS_SIZE                   (364)

// "Locked" array of 30 integers
// "point" array of 30 x/y int pairs


/* --- navdata_vision_detect_t --- */
/* Position of all tags detected by vision */
#define NAV_VIS_DETECT_TAG                  (16)
#define NAV_VIS_DETECT_SIZE                 (120)

#define NAV_VIS_DETECT_NB_DETECTED          (4)

#define NAV_VIS_DETECT_TYPE1                (8)
#define NAV_VIS_DETECT_TYPE2                (12)
#define NAV_VIS_DETECT_TYPE3                (16)
#define NAV_VIS_DETECT_TYPE4                (20)

#define NAV_VIS_DETECT_XC1                  (24)
#define NAV_VIS_DETECT_XC2                  (28)
#define NAV_VIS_DETECT_XC3                  (32)
#define NAV_VIS_DETECT_XC4                  (36)

#define NAV_VIS_DETECT_YC1                  (40)
#define NAV_VIS_DETECT_YC2                  (44)
#define NAV_VIS_DETECT_YC3                  (48)
#define NAV_VIS_DETECT_YC4                  (52)

#define NAV_VIS_DETECT_WIDTH1               (56)
#define NAV_VIS_DETECT_WIDTH2               (60)
#define NAV_VIS_DETECT_WIDTH3               (64)
#define NAV_VIS_DETECT_WIDTH4               (68)

#define NAV_VIS_DETECT_HEIGHT1              (72)
#define NAV_VIS_DETECT_HEIGHT2              (76)
#define NAV_VIS_DETECT_HEIGHT3              (80)
#define NAV_VIS_DETECT_HEIGHT4              (84)

#define NAV_VIS_DETECT_DIST1                (88)
#define NAV_VIS_DETECT_DIST2                (92)
#define NAV_VIS_DETECT_DIST3                (96)
#define NAV_VIS_DETECT_DIST4                (100)

#define NAV_VIS_DETECT_ANGLE1               (104)
#define NAV_VIS_DETECT_ANGLE2               (108)
#define NAV_VIS_DETECT_ANGLE3               (112)
#define NAV_VIS_DETECT_ANGLE4               (116)


/* --- navdata_watchdog_t --- */
#define NAV_WATCHDOG_TAG                    (17)
#define NAV_WATCHDOG_SIZE                   (8)


/* --- navdata_adc_data_frame_t --- */
#define NAV_ADC_DATA_TAG                    (18)
#define NAV_ADC_DATA_SIZE                   (40)


/* --- navdata_video_stream_t --- */
#define NAV_VIDEO_STREAM_TAG                (19)
#define NAV_VIDEO_STREAM_SIZE               (29)


/* --- navdata_games_t --- */
#define NAV_GAMES_TAG                       (20)
#define NAV_GAMES_SIZE                      (12)

/* --- navdata_pressure_raw_t --- */
#define NAV_PRESSURE_RAW_TAG                (21)
#define NAV_PRSSURE_RAW_SIZE                (18)

#define NAV_PRESSURE_RAW_UP                 (4)
#define NAV_PRESSURE_RAW_UT                 (8)
#define NAV_PRESSURE_RAW_TEMP               (10)
#define NAV_PRESSURE_RAW_PRESS              (14)

/* --- navdata_magneto_t --- */
#define NAV_MAGNETO_TAG                     (22)
#define NAV_MAGNETO_SIZE                    (75) //75

#define NAV_MAGNETO_MAG_X                   (22)
#define NAV_MAGNETO_MAG_Y                   (26)
#define NAV_MAGNETO_MAG_Z                   (30)
#define NAV_MAGNETO_HEADING                 (46) //46
#define NAV_MAGNETO_HEADING_GYRO            (50) //50
#define NAV_MAGNETO_HEADING_FUSION          (54) //54

/* --- navdata_kalman_pressure_t --- */
#define NAV_KALMAN_PRESSURE_TAG             (24)
#define NAV_KALMAN_PRESSURE_SIZE            (60)

#define NAV_KALMAN_OFFSET_PRESSURE          (4)
#define NAV_KALMAN_EST_Z                    (8)
#define NAV_KALMAN_OFFSET_US                (24)

/* --- checksum --- */
#define NAV_CHECKSUM_TAG                    (0xFFFF)
#define NAV_CHECKSUM_SIZE                   (8)


/* Other masks */
#define ARDRONE_FLY_MASK             (1 <<  0)  /*!< FLY MASK : (0) ardrone is landed, (1) ardrone is flying */
#define ARDRONE_VIDEO_MASK           (1 <<  1)  /*!< VIDEO MASK : (0) video disable, (1) video enable */
#define ARDRONE_VISION_MASK          (1 <<  2)  /*!< VISION MASK : (0) vision disable, (1) vision enable */
#define ARDRONE_CONTROL_MASK         (1 <<  3)  /*!< CONTROL ALGO : (0) euler angles control, (1) angular speed control */
#define ARDRONE_ALTITUDE_MASK        (1 <<  4)  /*!< ALTITUDE CONTROL ALGO : (0) altitude control inactive (1) altitude control active */
#define ARDRONE_USER_FEEDBACK_START  (1 <<  5)  /*!< USER feedback : Start button state */
#define ARDRONE_COMMAND_MASK         (1 <<  6)  /*!< Control command ACK : (0) None, (1) one received */
#define ARDRONE_CAMERA_MASK          (1 <<  7)  
#define ARDRONE_TRAVELLING_MASK      (1 <<  8)  
#define ARDRONE_USB_MASK             (1 <<  9) 
#define ARDRONE_NAVDATA_DEMO_MASK    (1 << 10) /*!< Navdata demo : (0) All navdata, (1) only navdata demo */
#define ARDRONE_NAVDATA_BOOTSTRAP    (1 << 11) /*!< Navdata bootstrap : (0) options sent in all or demo mode, (1) no navdata options sent */
#define ARDRONE_MOTORS_MASK          (1 << 12) /*!< Motors status : (0) Ok, (1) Motors problem */
#define ARDRONE_COM_LOST_MASK        (1 << 13) /*!< Communication Lost : (1) com problem, (0) Com is ok */
#define ARDRONE_SOFTWARE_FAULT       (1 << 14)
#define ARDRONE_VBAT_LOW             (1 << 15) /*!< VBat low : (1) too low, (0) Ok */
#define ARDRONE_USER_EL              (1 << 16) /*!< User Emergency Landing : (1) User EL is ON, (0) User EL is OFF*/
#define ARDRONE_TIMER_ELAPSED        (1 << 17) /*!< Timer elapsed : (1) elapsed, (0) not elapsed */
#define ARDRONE_MAGNETO_NEEDS_CALIB  (1 << 18)
#define ARDRONE_ANGLES_OUT_OF_RANGE  (1 << 19) /*!< Angles : (0) Ok, (1) out of range */
#define ARDRONE_WIND_MASK            (1 << 20)
#define ARDRONE_ULTRASOUND_MASK      (1 << 21) /*!< Ultrasonic sensor : (0) Ok, (1) deaf */
#define ARDRONE_CUTOUT_MASK          (1 << 22) /*!< Cutout system detection : (0) Not detected, (1) detected */
#define ARDRONE_PIC_VERSION_MASK     (1 << 23) /*!< PIC Version number OK : (0) a bad version number, (1) version number is OK */
#define ARDRONE_ATCODEC_THREAD_ON    (1 << 24) /*!< ATCodec thread ON : (0) thread OFF (1) thread ON */
#define ARDRONE_NAVDATA_THREAD_ON    (1 << 25) /*!< Navdata thread ON : (0) thread OFF (1) thread ON */
#define ARDRONE_VIDEO_THREAD_ON      (1 << 26) /*!< Video thread ON : (0) thread OFF (1) thread ON */
#define ARDRONE_ACQ_THREAD_ON        (1 << 27) /*!< Acquisition thread ON : (0) thread OFF (1) thread ON */
#define ARDRONE_CTRL_WATCHDOG_MASK   (1 << 28) /*!< CTRL watchdog : (1) delay in control execution (> 5ms), (0) control is well scheduled */
#define ARDRONE_ADC_WATCHDOG_MASK    (1 << 29) /*!< ADC Watchdog : (1) delay in uart2 dsr (> 5ms), (0) uart2 is good */
#define ARDRONE_COM_WATCHDOG_MASK    (1 << 30) /*!< Communication Watchdog : (1) com problem, (0) Com is ok */
#define ARDRONE_EMERGENCY_MASK       (1 << 31)  /*!< Emergency landing : (0) no emergency, (1) emergency */

#define MYKONOS_ANGLES_OUT_OF_RANGE  (1 << 19) //!< Angles : (0) Ok, (1) out of range 
#define MYKONOS_WIND_MASK            (1 << 20) //!< Wind : (0) Ok, (1) too much to fly 
#define MYKONOS_ULTRASOUND_MASK      (1 << 21) //!< Ultrasonic sensor : (0) Ok, (1) deaf 
#define MYKONOS_CUTOUT_MASK          (1 << 22) //!< Cutout system detection : (0) Not detected, (1) detected

#define MYKONOS_COM_WATCHDOG_MASK    (1 << 30) //!< Communication Watchdog : (1) com problem, (0) Com is ok 
#define MYKONOS_EMERGENCY_MASK       (1 << 31) //!< Emergency landing : (0) no emergency, (1) emergency 





void 		print_navdata_state();
uint32_t 	getDroneState() ;
bool 		getStateFlyMask();
bool 		getStateBattery();
bool 		getStateUserEmergencyLanding();
bool 		getMaskAltitudeControlAlgo();
bool 		getStateEmergencyLanding();
bool 		getStateMagnetometer();
#endif // _NAVDATA_H_
