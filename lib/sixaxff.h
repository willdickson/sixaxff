/*---------------------------------------------------------------------
  sixaxff
  Copyright (C) William Dickson, 2008.
  
  wbd@caltech.edu
  www.willdickson.com

  Released under the LGPL Licence, Version 3
  
  This file is part of sixaxff.

  sixaxff is free software: you can redistribute it and/or modify it
  under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation, either version 3 of the
  License, or (at your option) any later version.
    
  sixaxff is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
  or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General
  Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with sixaxff.  If not, see
  <http://www.gnu.org/licenses/>.

----------------------------------------------------------------------
  sixaxff.h

  Purpose: Contains preprocessor directives, declarations and 
  functions definitions for the sixaxff library.
 
  Author: Will Dickson
---------------------------------------------------------------------- */
#ifndef INC_YAWFF_H_
#define INC_YAWFF_H_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <sys/mman.h>
#include <rtai_lxrt.h>
#include <rtai_comedi.h>
#include <signal.h>

#define TRUE 1
#define FALSE 0
#define SUCCESS 0
#define FAIL -1
#define DIO_HI 1
#define DIO_LO 0
#define NUM_COMEDI_DEV 2
#define MAX_MOTOR 10
#define NUM_MOTOR_W_CTLR_ON 7
#define MAX_DIO 23
#define MAX_AIN 15
#define ERR_SZ 200
#define MAX_DT_NS 10000000         // 100 Hz 
#define MIN_DT_NS 100000           // 10 kHz
#define CLOCK_HI_NS 60000          // Time in ns for which clock is high 
#define RAD2DEG (180.0/M_PI)       // Convert radians to degrees
#define DEG2RAD (M_PI/180.0)       // Convert degrees to radians
#define NS2S 1.0e-9                // Convert nanoseconds to seconds 
#define S2NS (1.0/NS2S)            // Convert seconds to nanoseconds
#define MIN_TORQ_LIM 0.0           // Minimum allowed torque limit Nm
#define MAX_TORQ_LIM 10.0          // Maximum allowed torque limit Nm
#define MAX_YAW (DEG2RAD*10000.0)    // Maximum allowed yaw position
#define MIN_YAW (DEG2RAD*(-10000.0)) // Maximum allowed yaw position
#define FF_ON 0                    // Force-feedback on
#define FF_OFF 1                   // Force-feedback off
#define CTLR_ON 0                  // Controller on
#define CTLR_OFF 1                 // Controller off

#define INTEG_EULER 0      // Const for integration by Euler method 
#define INTEG_RKUTTA 1     // Const for integration by Runge-Kutta
#define INTEG_UNKNOWN 2    // Const for unknown integration method 
                           //(used for unit testing)
#define EMPTY_ARRAY 0      // Indicates array type is empty
#define INT_ARRAY 1        // Indicates array type ingeter
#define FLT_ARRAY 2        // Indicates array type float
#define DBL_ARRAY 3        // Indicates array type double
#define UNKNOWN_ARRAY 4    // Indicates array of unkown type
                           //(use for unit testing)
#define AIN_ZERO_DT_MIN 0.0005   // Minimum allowed zeroing interval
#define AIN_RANGE 0              // Analog input range
#define AIN_AREF AREF_GROUND     // Analog input reference

#define RT_TASK_ERROR 2   // Mask used to detect if an error occured in the realtime thread
#define RT_TASK_SIGINT 4  // Mask used to detect if an sigint stopped the realtime thread


typedef void (*sighandler_t)(int);

// Structure for numpy array
typedef struct {
  void *data;
  int nrow;
  int ncol; 
  int s0; 
  int s1;
  int type;
} array_t;


// Structure for configuration
typedef struct {
  char *dev_name[NUM_COMEDI_DEV]; // Comedi device name  
  unsigned int ain_dev;    // Device used for analog input
  unsigned int dio_dev;    // Device used for digital io
  unsigned int ain_subdev; // Analog input subdevice
  unsigned int dio_subdev;          // Digital IO subdevice
  int num_motor;           // Total number of motors
  int yaw_motor;           // Yaw motor number
  int dio_clk[MAX_MOTOR];  // DIO clock pins
  int dio_dir[MAX_MOTOR];  // DIO direction pins
  int kine_map[MAX_MOTOR]; // Map from kinematics to motors
  int motor_id_map[MAX_MOTOR]; // Map from kinematics to motor identifier 
  int dio_disable;         // Yaw motor disable pin 
  unsigned int yaw_ain;    // Analog input channel for yaw torque
  float yaw_ain_zero_dt;   // Sampling interval for zeroing ain 
  unsigned int yaw_ain_zero_num;  // Number of zeroing sample points 
  float yaw_volt2torq;     // Voltage to torque conversion
  float yaw_inertia;       // Moment of inertia about yaw axis
  float yaw_ind2deg;       // Index to degree conversion for yaw
  float yaw_torq_lim;      // Yaw torque limit (Nm)
  float yaw_torq_deadband; // Yaw torque beadband (Nm)
  float yaw_filt_lpcut;    // Yaw torque lowpass filter cutoff (Hz)
  float yaw_filt_hpcut;    // Yaw torque highpss filter cutoff (Hz)
  float yaw_damping;       // Damping constant for yaw axis
  int dt;                  // Realtime loop timestep (ns)
  int integ_type;          // Integrator type
  float startup_t;         // Startup window in which torque is set to zero
  int ff_flag;             // Sets force-feedback on or off
} config_t;

// Structure for dynamic state
typedef struct {
  float pos;  // Yaw axis position (rad)
  float vel;  // Yaw axis velocity (rad/s)
} state_t;

// Structure for return data
typedef struct {
  array_t t;     // Time (s)
  array_t pos;   // Yaw axis position (rad)
  array_t vel;   // Yaw axis velocity (rad/s)
  array_t torq;  // Torque (Nm), column 1 filtered, column 1 raw.
} data_t;

// Structure for comedi device information 
typedef struct {
  void *device;
  comedi_krange krange;
  int maxdata;
} comedi_info_t;

// Structure for torque data
typedef struct {
  float zero;       // Torque sensor zero
  float last;       // Last filtered (both high and low pass) torque measurement (Nm)
  float std;        // Torque sensor standard deviation
  float raw;        // Last raw torque measurement (Nm) 
  float highpass;   // Intermediate highpass filtered torque
} torq_info_t;

// Six axis force-feedback function 
extern int sixaxff( 
    array_t kine, 
    config_t config, 
    data_t data, 
    int end_pos[]
    ); 

// Cleanup fucntion for realtime thread 
extern int rt_cleanup(
    int level, 
    comedi_info_t comedi_info[], 
    RT_TASK *rt_task
    );

// Initialize comedi device based on configuration
extern int init_comedi(
    comedi_info_t comedi_info[], 
    config_t config
    );

// Get zero value for yaw torque analog input channel
extern int get_ain_zero(
    comedi_info_t comedi_info[],  
    config_t config, 
    float *ain_zero,
    float *ain_std
     );

// Get zero value for yaw torque 
extern int get_torq_zero(
    comedi_info_t comedi_info[], 
    config_t config, 
    float *torq_zero,
    float *torq_std
    );

// Read yaw torque sensor analog inout
extern int get_ain(
    comedi_info_t comedi_info[], 
    config_t config, 
    float *ain
    );

// Read torque from yaw torque sensor
extern int get_torq(
    comedi_info_t comedi_info[], 
    config_t config, 
    float *torq
    );

// Convert analog input value to voltage
extern int ain_to_phys(
    lsampl_t data, 
    comedi_info_t comedi_info, 
    float *volts
    );

// Update yaw dynamics state vector one timestep
extern int update_state( 
    state_t *state, 
    double t,
    torq_info_t *torq_info, 
    comedi_info_t comedi_info[], 
    config_t config
    );

// Initialize motor indices to zeros
extern void init_ind(int motor_ind[][2], config_t config);

// Update motor indices one timestep
extern int update_ind(
    int motor_ind[][2], 
    array_t kine, 
    int kine_ind, 
    state_t *state, 
    config_t config
    );

// Update state estimate
extern state_t get_state_est(
    state_t *state,
    data_t data,
    int ind,
    int delay
    );

// Update motor positions - move the motors
extern int update_motor(
    int motor_ind[][2], 
    comedi_info_t comedi_info[], 
    config_t config
    );

// Update data (t,pos,vel,torq) and index ind 
extern int update_data(
    data_t data, 
    int ind, 
    double t, 
    state_t *state, 
    torq_info_t torq_info
    ); 

// Set clock dio lines to DIO_LO
extern int set_clks_lo(
    comedi_info_t comedi_info[], 
    config_t config
    );

// Reassign sigint signal handler
extern sighandler_t reassign_sigint(sighandler_t sigint_func);

#endif // INC_YAWFF_H_

