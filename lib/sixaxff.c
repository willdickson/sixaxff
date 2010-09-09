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
  sixaxff.c

  Purpose: 

  Author: Will Dickson 
---------------------------------------------------------------------- */
#include <unistd.h>
#include <time.h>
#include "sixaxff.h"
#include "util.h"
#include "check.h"

// Names and units for sixaxis sensor data
const char *FT_NAMES[] = {"fx", "fy", "fz", "tx", "ty", "tz"};
const char *FT_UNITS[] = {"N", "N", "N", "Nm", "Nm", "Nm"};

// RT-task parameters
#define PRIORITY 1
#define STACK_SIZE 4096
#define MSG_SIZE 0

// RT-task cleanup levels
#define RT_CLEANUP_LEVEL_1 1
#define RT_CLEANUP_LEVEL_2 2
#define RT_CLEANUP_ALL 2

// Values specifying whether or not to wait on a lock 
#define RT_LOCK_WAIT 0
#define RT_LOCK_NOWAIT 1

// Values specifying whether RT task is running
#define RT_STOPPED 0
#define RT_RUNNING 1

// Structure for arguments passed to sixaxff real-time thread
typedef struct {
    array_t *kine;
    config_t *config;
    data_t *data;
} sixaxff_args_t;

// Structure for motor information
typedef struct {
  int pos;
  int vel;
} motor_info_t;

// Struture for reporting status information 
typedef struct {
  int ind;
  double t;
  float pos[NUM_FF];
  float vel[NUM_FF];
  float ft[NUM_FF];
  int motor_ind[MAX_MOTOR];
  int running;
  int err_flag;
  SEM* lock;
} status_t;
  
// Function prototypes
static void *sixaxff_thread(void *args);
void sigint_func(int sig);
void init_status_vals(status_t *status);
void read_status(status_t *status_copy);
void update_status(
        int i, 
        float t, 
        state_t state[], 
        ft_info_t ft_info,
        unsigned int ff_ft[],
        motor_ind_t motor_ind[],
        int running,
        int err_msg,
        int wait_flag);

// Global variables and constants
volatile static int end = 0;
static status_t status;

// -------------------------------------------------------------------
// Function: sixaxff
//
// Puropse: Main function for yaw turn force-feedback task. Spawns
// real-time thread to handle kinematics outscan, data acquisition, 
// and yaw dynamics. During outscaning displays information regarding
// ongoing real-time task.   
//
// Arguments:
//   
//  kine     = wing kinematics array
//  config   = system configuration structure
//  data     = structure of return data arrays
//  end_pod  = pointer to final position in motor ind
//  
// Return: SUCCESS or FAIL
//
// -------------------------------------------------------------------
int sixaxff(array_t kine, config_t config, data_t data, int end_pos[])
{
    RT_TASK *sixaxff_task;
    int rt_thread;
    sighandler_t sighandler = NULL;
    int rtn_flag = SUCCESS;
    sixaxff_args_t thread_args;
    status_t status_copy;
    struct timespec sleep_ts;
    int i;

    // Initialize globals
    end = 0;
    init_status_vals(&status); // Values only - doesn't initialize lock

    // Startup method 
    printf("\n");
    printf("                  Starting sixaxff \n");
    printf("=======================================================\n");

    // Check inputs
    fflush_printf("checking input args\n");
    if (check_sixaxff_input(kine,config,data) != SUCCESS) {
        PRINT_ERR_MSG("bad input data");
        return FAIL;
    }
    print_config(config);

    // Setup SIGINT handler
    fflush_printf("reassigning SIGINT handler\n");
    sighandler = reassign_sigint(sigint_func);
    if (sighandler == SIG_ERR) {
        return FAIL;
    }

    // Intialize status semephore
    //  ------------------------------------------------------------
    // Note, if lxrt stuff is not running I get segfault here
    // Is there any way to test if the lxrt stuff in inserted into 
    // the kernel???
    // -------------------------------------------------------------
    fflush_printf("initializing status semaphore\n");
    status.lock = rt_typed_sem_init(nam2num("STATUS"),1,BIN_SEM | FIFO_Q);
    if (status.lock == NULL) {
        PRINT_ERR_MSG("unable to initialize status semaphore");
        fflush_printf("restoring SIGINT handler\n");
        sighandler = reassign_sigint(sighandler);
        return FAIL;
    }


    //Initialize RT task
    fflush_printf("initializing sixaxff_task\n");
    rt_allow_nonroot_hrt();
    sixaxff_task = rt_task_init(nam2num("YAWFF"),PRIORITY,STACK_SIZE,MSG_SIZE);
    if (!sixaxff_task) {
        PRINT_ERR_MSG("error initializing sixaxff_task");
        fflush_printf("restoring SIGINT handler\n");
        sighandler = reassign_sigint(sighandler);
        return FAIL;
    }
    rt_set_oneshot_mode();
    start_rt_timer(0);

    // Assign arguments to thread
    thread_args.kine = &kine;
    thread_args.config = &config;
    thread_args.data = &data;

    // Start motor thread
    fflush_printf("starting rt_thread\n");
    rt_thread = rt_thread_create(sixaxff_thread, ((void *)&thread_args), STACK_SIZE);

    // Set reporter sleep timespec
    sleep_ts.tv_sec = 0;
    sleep_ts.tv_nsec = 100000000;

    // Run time display
    do {
        // copy of status information structure - use locks
        read_status(&status_copy);

        // Once realtime task is running display data
        if (status_copy.running) {
            fflush_printf("                                                                            "); 
            fflush_printf("\r");
            fflush_printf("%3.0f\%, t: %3.2f, ", 100.0*(float)status.ind/(float)kine.nrow, status_copy.t);
            for (i=0; i<NUM_FF; i++) {
                fflush_printf("p[%d]: %3.2f(%s), v[%d]: %3.2f(%s/s), ft[%d]: %s %3.2f(%s)",
                        i,
                        status_copy.pos[i]*config.ff_ind2unit[i],
                        config.ff_axesunits[i],
                        i,
                        status_copy.vel[i]*config.ff_ind2unit[i],
                        config.ff_axesunits[i],
                        i, 
                        FT_NAMES[config.ff_ft[i]],
                        status_copy.ft[i],
                        FT_UNITS[config.ff_ft[i]]
                    );
                if (i<NUM_FF-1) {
                    fflush_printf(", ");
                }
            }
            fflush_printf("\r");
        }
        nanosleep(&sleep_ts,NULL);
    } while (!end);

    // Wait to join
    rt_thread_join(rt_thread);
    fflush_printf("rt_thread joined\n");

    // One last read of status to get errors and final position
    read_status(&status_copy); 

    // Clean up
    stop_rt_timer();
    rt_task_delete(sixaxff_task);
    fflush_printf("sixaxff_task deleted\n");
    rt_sem_delete(status.lock);


    // Restore old SIGINT handler
    fflush_printf("restoring SIGINT handler\n");
    sighandler = reassign_sigint(sighandler);
    if (sighandler == SIG_ERR) {
        PRINT_ERR_MSG("restoring signal handler failed");
        rtn_flag = FAIL;
    }


    // Print any error messages
    if (status_copy.err_flag & RT_TASK_SIGINT) {
        fflush_printf("real-time task stopped with: RT_SIGINT\n");
    }

    if (status_copy.err_flag & RT_TASK_ERROR) {
        fflush_printf("real-time task stopped with: RT_ERROR \n");
    }

    // Copy motor indices into end_pos
    fflush_printf("end_pos: ");
    for (i=0; i<config.num_motor; i++) {
        end_pos[i] = status_copy.motor_ind[i];
        fflush_printf("%d", end_pos[i]);
        if (i!=config.num_motor-1) fflush_printf(", ");
    }
    fflush_printf("\n");

    // Temporary
    return rtn_flag;
}

// ------------------------------------------------------------------
// Function: sixaxff_thread
//
// Purpose: Realtime thread for sixaxff function. Performs real-time yaw 
// turn force-feedback task which consists of:
//
// 1.) Outscanning wing kinematics,
// 2.) Acquiring data from torque sensor
// 3.) Controlling yaw dynamics using acquired torque 
//
// Arguments:
//   args = pointer to thread_args structure.
// 
// Return: void.
//
// ------------------------------------------------------------------

static void *sixaxff_thread(void *args)
{
    RT_TASK *rt_task=NULL;
    RTIME now_ns;
    sixaxff_args_t *thread_args=NULL;
    comedi_info_t comedi_info[NUM_COMEDI_DEV];
    array_t kine;
    config_t config;
    data_t data;
    ft_info_t ft_info;
    state_t state[NUM_FF];
    motor_ind_t motor_ind[MAX_MOTOR];
    float runtime;
    double t;
    int err_flag = 0;
    int i;

    // Unpack arguments passed to thread
    thread_args = (sixaxff_args_t *) args;
    kine = *(thread_args -> kine);
    config = *(thread_args -> config);
    data = *(thread_args -> data);


    // Initialize, time, dynamic state, and motor indices
    for (i=0; i<NUM_FF; i++) {
        state[i].pos = 0.0;
        state[i].vel = 0.0;
        state[i].pos_prev = 0.0;
        state[i].vel_prev = 0.0;
    }  
    init_ind(motor_ind,config);
    t = 0.0;

    // Initialize torque info, read sensor calibration
    if (init_ft_info(&ft_info, config) != SUCCESS) {
        PRINT_ERR_MSG("unable to initialize ft_info");
        end = 1;
        return 0;
    }

    // Initialize comedi device
    if (init_comedi(comedi_info, config) != SUCCESS) {
        PRINT_ERR_MSG("unable to initialize comedi device");
        end = 1;
        return 0;
    }

    // Set sensor zero
    if (set_sensor_zero(comedi_info, config, &ft_info) != SUCCESS) {
        PRINT_ERR_MSG("unable to set sensor zero");
        end = 1;
        return 0;
    }

    // Initialize rt_task
    fflush_printf("Initializing rt_task \n");
    rt_task = rt_task_init_schmod(nam2num("MOTOR"),0, 0, 0, SCHED_FIFO, 0xF);
    if (!rt_task) {          
        PRINT_ERR_MSG("cannot initialize rt task");
        // Error, clean up and exit
        if (rt_cleanup(RT_CLEANUP_LEVEL_1,comedi_info,rt_task) != SUCCESS) {
            PRINT_ERR_MSG("rt_cleanup failed");
        }
        end = 1;
        return 0;
    }
    rt_task_use_fpu(rt_task,1); // Enable use of floating point math

    // Go to hard real-time
    runtime = ((float) config.dt)*((float) kine.nrow)*NS2S;
    fflush_printf("starting hard real-time, T = %1.3f(s)\n", runtime);
    mlockall(MCL_CURRENT|MCL_FUTURE);
    rt_make_hard_real_time();

    // Loop over kinematics 
    for (i=0; i<kine.nrow; i++) {

        now_ns = rt_get_time_ns();

        // Update dynamic state
        if (update_state(state, t, &ft_info, comedi_info,config) != SUCCESS) {
            PRINT_ERR_MSG("updating dynamic state failed");
            err_flag |= RT_TASK_ERROR;
            break;
        }

        ///////////////////////////////////////////////////////////
        // Update motor index array
        if (update_ind(motor_ind,kine,i,state,config) != SUCCESS) {
            PRINT_ERR_MSG("updating motor indices failed");
            err_flag |= RT_TASK_ERROR;
            break;
        }
        ///////////////////////////////////////////////////////////

    /*
    // Update motor positions
    if (update_motor(motor_ind, comedi_info, config) != SUCCESS) {
    PRINT_ERR_MSG("updating motor positions failed");
    err_flag |= RT_TASK_ERROR;
    break;
    }

    // Update data
    if (update_data(data,i,t,state,torq_info) != SUCCESS) {
    PRINT_ERR_MSG("updating data arrays failed");
    err_flag |= RT_TASK_ERROR;
    break;
    }

    // Sleep for CLOCK_HI_NS and then set clock lines low
    rt_sleep_until(nano2count(now_ns + (RTIME) CLOCK_HI_NS));
    if (set_clks_lo(comedi_info, config) != SUCCESS) {
    PRINT_ERR_MSG("setting dio clks failed");
    err_flag |= RT_TASK_ERROR;
    break;
    }

    */

        // Check if end has been set to 1 by SIGINT handler
        if (end == 1) {
            fflush_printf("\nSIGINT - exiting real-time");
            err_flag |= RT_TASK_SIGINT;
            break;
        }

        // Update information if global variable status 
        update_status(i,t,state,ft_info,config.ff_ft,motor_ind,RT_RUNNING,0,RT_LOCK_NOWAIT);

        // Sleep until next period
        rt_sleep_until(nano2count(now_ns + config.dt));

        // Update time
        t += NS2S*((double)config.dt);
    } // End for i

    // Set status information to final values before exiting
    update_status(
    kine.nrow-1,
    t,
    state,
    ft_info,
    config.ff_ft,
    motor_ind,
    RT_STOPPED,
    err_flag,
    RT_LOCK_WAIT
    );


    // Leave realtime
    rt_make_soft_real_time();
    munlockall();
    fflush_printf("\nleaving hard real-time\n");


    // Clean up
    if (rt_cleanup(RT_CLEANUP_ALL, comedi_info, rt_task)!=SUCCESS) {
        PRINT_ERR_MSG("rt_cleanup failed");
    }

    end = 1;

    return 0;
}


// -------------------------------------------------------------------
// Function: init_status_vals
//
// Purpose: initializes variable of type status_t. No locks are used
// during initialization to access the structure. 
//
// NOTE: this does not initialize the semaphore lock. This must be
// done separately.
//
// Arguments:
//   status = pointer to status_t structure
//
// -------------------------------------------------------------------
void init_status_vals(status_t *status)
{
  int i;
  status -> ind = 0;
  status -> t = 0.0;
  for (i=0; i<NUM_FF; i++) {
      (status -> pos)[i] = 0.0;
      (status -> vel)[i] = 0.0;
      (status -> ft)[i] = 0.0;
  }
  status -> running = RT_STOPPED;
  status -> err_flag = 0;
  for (i=0; i<MAX_MOTOR; i++) {
    (status -> motor_ind)[i] = 0;
  }
  return;
}

// --------------------------------------------------------------------
// Function: read_status
//
// Purpose: Reads data in global variable status.
//
// Arguments:
//   status_copy = pointer to local copy of status structure.
//
// --------------------------------------------------------------------
void read_status(status_t *status_copy)
{
  rt_sem_wait(status.lock);
  *status_copy = status;  
  rt_sem_signal(status.lock);
  return;
}

// --------------------------------------------------------------------
// Function: update_status
//
// Purpose: updates data in global variable status which is used
// for reporting runtime data to the user via the main thread.
//
// Arguments:
//
//   i          = current kinematics index
//   t          = current time in seconds
//   state      = current state array
//   ft_info    = current force/torque information strucuture 
//   ff_ft      = array of indices specifiying sensor channels used 
//                for force/torque feedback 
//   running    = flag which indicates real-time loop is running
//   err_msg    = error indocator
//   wait_flag  = RT_LOCK_WAIT or RT_LOCK_NOWAIT
//
// Return: void
//
// ---------------------------------------------------------------------

void update_status(
        int i, 
        float t, 
        state_t state[], 
        ft_info_t ft_info,
        unsigned int ff_ft[],
        motor_ind_t motor_ind[],
        int running,
        int err_flag,
        int wait_flag
        )
{
    int j;
    int have_lock;

    switch (wait_flag) {

        case RT_LOCK_NOWAIT:
            // Try and acquire lock - but don't wait if you can't get it
            have_lock = rt_sem_wait_if(status.lock);
            break;

        case RT_LOCK_WAIT:
            // Acquire lock - wait if you can't get it. 
            rt_sem_wait(status.lock);
            have_lock = 1;
            break;

        default:
            // Unknown flag
            PRINT_ERR_MSG("unkown wait_flag");
            return;
            break;
    }

    // Update data in status structure if lock was aqcuired 
    if (have_lock) {
        status.ind = i;
        status.t = t;
        for (j=0; j<NUM_FF; j++) {
            status.pos[j] = state[j].pos;
            status.vel[j] = state[j].vel;
            status.ft[j] = ft_info.ft_last[ff_ft[j]]; 
        }
        status.running = running;
        status.err_flag |= err_flag;
        for (j=0;j<MAX_MOTOR;j++) {
            status.motor_ind[j] = motor_ind[j].curr;
        }
        rt_sem_signal(status.lock);
    }
    return;
}

// ---------------------------------------------------------------------
// Function: update_data
//
// Purpose: write new time, position, velocity, and torq data to data 
// array structure.
//
// Arguments:
//   data      = structure of data arrays
//   ind       = index at which to place new data
//   t         = current outscan time in seconds
//   state     = yaw dynamics state vector structure
//   torq_info = torque information  
// 
// return: SUCCESS or FAIL
//
// --------------------------------------------------------------------- 

/*
int update_data(
    data_t data, 
    int ind, 
    double t,
    state_t *state, 
    torq_info_t torq_info
    )
{
  if (set_array_val(data.t,ind,0,&t) != SUCCESS) {
    PRINT_ERR_MSG("setting time array value failed");
    return FAIL;
  }
  if (set_array_val(data.pos,ind,0,&state[1].pos) != SUCCESS) {
    PRINT_ERR_MSG("setting pos array value failed");
    return FAIL;
  }
  if (set_array_val(data.vel,ind,0,&state[1].vel) != SUCCESS) {
    PRINT_ERR_MSG("setting vel array value failed");
    return FAIL;
  }
  if (set_array_val(data.torq,ind,0,&torq_info.last) != SUCCESS) {
    PRINT_ERR_MSG("setting torq array value failed");
    return FAIL;
  }
  if (set_array_val(data.torq,ind,1,&torq_info.raw) != SUCCESS) {
      PRINT_ERR_MSG("setting torq array value failed");
      return FAIL;
  }
  return SUCCESS;
}
*/


// ---------------------------------------------------------------------
// Function: set_clks_lo
//
// Purpose: set clock dio lines to DIO_LO.
//
// Arguments:
//   comedi_info = structure of comedi daq/dio device information/
//   config      = system configuration structure.
//
// Return: SUCCESS or FAIL
//
// ---------------------------------------------------------------------

/*
int set_clks_lo(comedi_info_t comedi_info[], config_t config)
{
  int rval;
  int i;

  for (i=0; i<config.num_motor; i++) {
    rval = comedi_dio_write(
        comedi_info[config.dio_dev].device,
        config.dio_subdev,
        config.dio_clk[i],
        DIO_LO
        );
    if (rval != 1) {
      PRINT_ERR_MSG("comedi_dio_write failed");
      return FAIL;
    }
  }
  return SUCCESS;
}
*/

// ---------------------------------------------------------------------
// Function: update_motor
//
// Purpose: Updates the position of motors based on the (updated) motor 
// index array.
//
// Arguments: 
//   motor_ind   = array of motor indices for current and previous time 
//                 steps.  motor[i][j] where i is motor # and j = 0 is 
//                 previous time step and j = 1 is current time step.
//   comedi_info = structure of comedi daq/dio card information.
//   config      = system configuration structure.
// 
// Return: SUCCESS or FAIL
//
// ---------------------------------------------------------------------

/*
int update_motor(
    int motor_ind[][2], 
    comedi_info_t comedi_info[], 
    config_t config
    )
{
  int i;
  int dpos;
  int dir_val;
  int rval;

  for (i=0; i<config.num_motor; i++) {
    
    dpos = motor_ind[i][1] - motor_ind[i][0];
    
    // Set direction DIO
    dir_val = dpos >= 1 ? DIO_HI : DIO_LO;
    rval = comedi_dio_write(
        comedi_info[config.dio_dev].device,
        config.dio_subdev,
        config.dio_dir[i],
        dir_val
        );
    if (rval!=1) {
      PRINT_ERR_MSG("comedi write dio dir failed");
      return FAIL;
    }

    // Set clock DIO
    if (abs(dpos) > 0) {
      rval = comedi_dio_write(
          comedi_info[config.dio_dev].device,
          config.dio_subdev,
          config.dio_clk[i],
          DIO_HI
          );
      if (rval != 1) {
        PRINT_ERR_MSG("conedi write dio clk failed");
        return FAIL;
      }
    }
  } // End for i

  return SUCCESS;
}
*/

// ---------------------------------------------------------------------
// Function: update_ind
//
// Purpose: Updates array of motor indices based on kinematics, 
// kinematics  index, and the dynamic state (for the yaw motor).
//
// Arguments:
//   motor_ind = array of motor indices for current and previous time 
//               steps.  motor[i][j] where i is motor # and j = 0 is 
//               previous time step and j = 1 is current time step.
//   kine      = kinematics array
//   kine_ind  = current kinematics index
//   state     = array of dynamic states (previous and current) for the
//               yaw motor.
//   config    = system configuration structure.
//
// Return: SUCCESS or FAIL
//
// ---------------------------------------------------------------------

/*
int update_ind(
    int motor_ind[][2],
    array_t kine, 
    int kine_ind,
    state_t *state, 
    config_t config
    )
{
  int i;
  int ind;
  int kine_num;
  int motor_num; 

  // Set current indices to previous indices 
  for (i=0; i<config.num_motor; i++) {
    motor_ind[i][0] = motor_ind[i][1];
  }

  // Set current index
  kine_num = 0;
  for (i=0; i<config.num_motor; i++) {
    if (i == config.yaw_motor)  {
      // This is the yaw motor 
      if (config.ff_flag == FF_ON) {
        // Force-feedback is on - get index from current state
        ind = (int)((RAD2DEG/config.yaw_ind2deg)*state[1].pos);
        motor_num = config.yaw_motor;
      }
      else {
        // Force-feedback is off - get index from kinematics
        motor_num = config.yaw_motor;
        if (get_array_val(kine,kine_ind,motor_num,&ind) != SUCCESS) {
          PRINT_ERR_MSG("problem accessing kine array");
          return FAIL;
        } 
        // Set position in state
        state[0].pos = config.yaw_ind2deg*DEG2RAD*ind;
        state[1].pos = config.yaw_ind2deg*DEG2RAD*ind;
      }
    }
    else { 
      // This is a wing motor
      motor_num = config.kine_map[kine_num];

      // Get index from kine array
      if (get_array_val(kine,kine_ind,motor_num,&ind) != SUCCESS) {
        PRINT_ERR_MSG("problem accessing kine array");
        return FAIL;
      }
      kine_num += 1;
    } 
    motor_ind[motor_num][1] = ind;
  }
  return SUCCESS;
}
*/


// ---------------------------------------------------------------------
// Function: init_ind
//
// Purpose: Initialize motor indices (previous and current)  to zero.
//
// Arguments:
//   motor_ind  = array of motor indices. Note, motor_ind[i][0] gives 
//                index for preivous time step for motor i, and 
//                motor_ind[i][1] gives index for current time step. 
//   config     = system configuration structure.
//
// Return: void
//  
// ---------------------------------------------------------------------
void init_ind(motor_ind_t motor_ind[], config_t config)
{
    int i,j; 

    for (i=0; i<config.num_motor; i++) {
        for (j=0; j<2; j++) {
            motor_ind[i].curr = 0;
            motor_ind[i].prev = 0;
        }
    }
    return;
}

// ---------------------------------------------------------------------
// Function: init_ft_info
//
// Purpose: Initialize structure containing force/torque information
//
// Arguments:
//   ft_info    = pointer to force/torque information structure
//   config     = system configuration structure.
//
// Return: SUCCESS/FAIL 
//  
// ---------------------------------------------------------------------
int init_ft_info(ft_info_t *ft_info, config_t config)
{
    int i;
    int rtn_val;

    for (i=0; i<6; i++) {
        (ft_info -> ain_zero)[i] = 0.0;
        (ft_info -> ain_std)[i] = 0.0;
        (ft_info -> ft_last)[i] = 0.0;
        (ft_info -> ft_raw)[i] = 0.0;
    }
    ft_info -> cal = NULL;
    fflush_printf("reading sensor calibration file: %s\n", config.cal_file_path);
    rtn_val = sixax_init_cal(&(ft_info -> cal), config.cal_file_path, config.ff_tooltrans);
    if (rtn_val == FAIL) {
        PRINT_ERR_MSG("unable to initialize six axis sensor calibration");
        return FAIL;
    }
    //sixax_print_calinfo(ft_info->cal);
    return SUCCESS;
} 

// ----------------------------------------------------------------------
// Function: update_state
//
// Purpose: Get reading from torque sensor and update dynamic state 
// structure. The raw torque reading is lowpass filtered and the lowpass
// filtered torque is checked against the system configuration torque 
// limit. If the abs value is too great an error occurs. 
//
// Arguments:
//   state        = array of dyanmic state vectors
//   t            = time in secs
//   torque_info  = torque information structure
//   comedi_info  = daq/dio device information structure
//   config       = system configuration structure
//
// Return: SUCCESS or FAIL
//
// ---------------------------------------------------------------------- 
int update_state(
        state_t state[], 
        double t,
        ft_info_t *ft_info,
        comedi_info_t comedi_info[], 
        config_t config
        )
{
    int i;
    int ff_ind;
    float dt;
    float ft_raw[6];
    float ft_filt[6];
    int rval;

    // Get time step in secs
    dt = ((float)config.dt)*NS2S;

    // Get data from  torque sensor and zero 
    if (get_ft(ft_info -> cal, comedi_info, config, ft_raw) != SUCCESS) {
        PRINT_ERR_MSG("error reading torque");
        return FAIL;
    }

  ///////////////////////////////////////////////////////////////////
  // Constant torque test  - set torque to some known value.
  //for (i=0; i<NUM_FF; i++) {
  //    ff_ind = config.ff_ft[i];
  //    ft_raw[ff_ind] = 0.001;
  //}
  ///////////////////////////////////////////////////////////////////
  

  // If we are inside start up wff_indow set the force feedback forces/torques to zero
  if (t < (double)config.startup_t) {
      for (i=0; i<NUM_FF; i++) {
          ff_ind = config.ff_ft[i];
          ft_raw[ff_ind] = 0.0;
      }
  }

  // Lowpass filter forces and torques 
  for (i=0; i<6; i++) {
      ft_filt[i] = lowpass_filt1(
              ft_raw[i], 
              (ft_info->ft_last)[i], 
              config.ain_filt_lpcut,
              dt
              );
  }

  // Update ft_info structure
  for (i=0; i<6; i++) {
      (ft_info -> ft_last)[i] = ft_filt[i];
      (ft_info -> ft_raw)[i] = ft_raw[i];
  }


  // Update dynamic state - only if force-feedback is turned on
  if (config.ff_flag == FF_ON) {

      for (i=0; i<NUM_FF; i++) {

          // Set previous state to current state
          state[i].pos_prev = state[i].pos;
          state[i].vel_prev = state[i].vel;

          // Integrate one time step
          ff_ind = config.ff_ft[i];
          rval = integrator(
                  &state[i],
                  ft_filt[ff_ind],
                  config.ff_mass[i], 
                  config.ff_damping[i], 
                  dt,
                  config.ff_integ_type
                  );
          if (rval != SUCCESS ) {
              PRINT_ERR_MSG("integrator failed");
              return FAIL;
          }
      }
  }

  return SUCCESS;
}

// -----------------------------------------------------------------------
// Function: set_sensor_zero 
//
// Purpose: Reads sample values from sensor and upades ft_info.cal 
// to automatically remove the bais values 
//
// Arguments:
//   comedi_info  = daq/dio device information structure
//   config       = system configuration structure
//   ft_info      = pointer to force/torque information structure
//
// Return: SUCCESS or FAIL
//
// -----------------------------------------------------------------------

int set_sensor_zero(
        comedi_info_t comedi_info[], 
        config_t config, 
        ft_info_t *ft_info)
{
    int rtn_val = SUCCESS;

    // Get analog input zero
    rtn_val = get_ain_zero(
            comedi_info,
            config,
            ft_info->ain_zero,
            ft_info->ain_std
            );
    if (rtn_val != SUCCESS) {
        PRINT_ERR_MSG("unable to get analog input zero");
        return FAIL;
    }

   // Set calibration bias values
   fflush_printf("setting sensor bias\n");
   rtn_val = sixax_set_bias(ft_info -> cal, ft_info -> ain_zero); 
   if (rtn_val != SUCCESS) {
       PRINT_ERR_MSG("unable to set sensor bias");
       return FAIL;
   }

    return SUCCESS;
}


// -----------------------------------------------------------------------
// Function: get_ain_zero
//
// Purpose: Determines the zero bias value in volts for the yaw torque
// analog input.
//
// Arguments:
//   comedi_info  = daq/dio device information structure
//   config       = system configuration structure
//   ain_zero     = pointer to float for analog input zero bias
//   ain_std      = pointer to float for analog input standard deviation
//
// Return: SUCCESS or FAIL
//
// -----------------------------------------------------------------------
int get_ain_zero(
        comedi_info_t comedi_info[], 
        config_t config, 
        float ain_zero[], 
        float ain_std[]
        )
{
    int i;
    int j;
    float ain[config.ain_zero_num][6];
    int ret_flag = SUCCESS;
    char err_msg[ERR_SZ];
    struct timespec sleep_req;
    float t_zero;

    t_zero = config.ain_zero_dt*config.ain_zero_num;
    fflush_printf("finding ain zero and std, T = %1.3f(s) \n", t_zero);

    // Set sleep timespec
    sleep_req.tv_sec = (time_t) config.ain_zero_dt;
    sleep_req.tv_nsec = 1.0e9*(config.ain_zero_dt - (time_t) config.ain_zero_dt);

    // Get samples for ain mean  
    for (i=0; i<config.ain_zero_num; i++) {

        if (get_ain(comedi_info, config, ain[i]) != SUCCESS) {
            snprintf(err_msg, ERR_SZ, "unable to read ain, i = %d", i);
            PRINT_ERR_MSG(err_msg);
            ret_flag = FAIL;
            break;
        }
        // Sleep for AIN_SLEEP_DT seconds
        nanosleep(&sleep_req, NULL);
    }

    // Compute ain mean - this is our zero
    for (i=0; i<6; i++) {
        ain_zero[i] = 0.0;
    }
    for (i=0; i<config.ain_zero_num;i++) {
        for (j=0; j<6; j++) {
            ain_zero[j] += ain[i][j];
        }
    }
    for (i=0; i<6; i++) {
        ain_zero[i] /= (float)config.ain_zero_num;
        fflush_printf("ain_zero[%d]: %f(V)\n", i, ain_zero[i]);
    }

    // Compute ain standard deviation
    for (i=0; i<6; i++) {
        ain_std[i] = 0.0;
    }
    for (i=0; i<config.ain_zero_num; i++) {
        for (j=0; j<6; j++) {
            ain_std[j] += powf(ain[i][j] - ain_zero[j], 2.0);
        }
    }
    for (i=0; i<6; i++) {
        ain_std[i] /= (float) config.ain_zero_num;
        ain_std[i] = sqrtf(ain_std[i]);
        fflush_printf("ain_std[%d]: %f(V)\n", i, ain_std[i]);
    }

    return ret_flag;
}

// ------------------------------------------------------------------
// Function: get_ain
//
// Prupose: Read yaw torq analog input voltage.
//
// Arguments:
//   comedi_info  = daq/dio device information structure
//   config       = system configuration structure
//   ain          = pointer to float for analog input value
//
// Return: SUCCESS or FAIL
//
// ------------------------------------------------------------------

int get_ain(comedi_info_t comedi_info[], config_t config, float ain[])
{

    int i;
    int rval;
    lsampl_t ain_lsampl; 

    // Loop over analog input channels
    for (i=0; i<6; i++) {
        // Read value from daq card, use comdei_data_read_delayed - to allow 
        // analog inputs to settle
        rval = comedi_data_read_delayed(
                comedi_info[config.ain_dev].device,
                config.ain_subdev,
                i,
                AIN_RANGE,
                AIN_AREF,
                &ain_lsampl,
                AIN_DELAY_NS
                );
        if (rval!=1) {
            PRINT_ERR_MSG("comedi_data_read failed");
            return FAIL;
        }

        // Convert integer analog input value to volts
        if (ain_to_phys(ain_lsampl, comedi_info[config.ain_dev], &ain[i]) != SUCCESS) {
            PRINT_ERR_MSG("ain_to_phys failed");
            return FAIL;
        }
    }
    return SUCCESS;
}

// ------------------------------------------------------------------
// Function: get_ft
//
// Purpose: Read forces and torques N and Nm from six axis sensor.
//
// Arguments:
//   comedi_info  = daq/dio device information structure
//   config       = system configuration structure
//   ft           = array for returning force and torque data 
//
// Return: SUCCESS or FAIL
//
// ------------------------------------------------------------------

int get_ft(Calibration *cal, comedi_info_t comedi_info[], config_t config, float ft[])
{

  float ain[6];
  
  // Read analog values from sensor
  if (get_ain(comedi_info, config, ain) != SUCCESS) {
    PRINT_ERR_MSG("unable to read analog input");
    return FAIL;
  }

  // Convert analog values to forces and torques
  sixax_sample2ft(cal,ain,ft);

  return SUCCESS;
}

// ------------------------------------------------------------------
// Function: init_comedi
//
// Purpose: Opens comedi device and sets clock, direction and disbale 
// dio lines to output. All lines are initially set to DIO_LO.
//
// Arguments:
//   comedi_info  = daq/dio device informtation structure
//   config       = system configuration structure.
//
// Return: SUCCESS or FAIL
//
// ------------------------------------------------------------------

int init_comedi(comedi_info_t comedi_info[], config_t config)
{
    int i;
    int rval;
    int dummy_ain_chan = 0;
    char err_msg[ERR_SZ];
    int ret_flag = SUCCESS;
    unsigned int ain_dev;
    unsigned int dio_dev;

    ain_dev = config.ain_dev;
    dio_dev = config.dio_dev;

    // Open comedi devices
    for (i=0; i<NUM_COMEDI_DEV; i++) {
        fflush_printf("opening %s\n",config.dev_name[i]);
        comedi_info[i].device = comedi_open(config.dev_name[i]);
        if (comedi_info[i].device == NULL) {
            snprintf(err_msg, ERR_SZ, "unable to open %s", config.dev_name[i]);
            PRINT_ERR_MSG(err_msg);
            return FAIL;
        }
    }

    // Configure dio by setting them to outputs
    fflush_printf("configuring dio lines\n");
    for (i=0; i<config.num_motor; i++) {    
        // Set clock lines to output
        rval = comedi_dio_config(
                comedi_info[dio_dev].device, 
                config.dio_subdev, 
                config.dio_clk[i],
                COMEDI_OUTPUT
                );
        if (rval != 1 ) {
            snprintf(err_msg, ERR_SZ, "unable to configure dio_clk[%d]", i);
            PRINT_ERR_MSG( err_msg);
            ret_flag = FAIL;
        }
        // Set direction lines to output 
        rval = comedi_dio_config(
                comedi_info[dio_dev].device, 
                config.dio_subdev, 
                config.dio_dir[i],
                COMEDI_OUTPUT
                );
        if (rval != 1) {
            snprintf(err_msg, ERR_SZ, "unable to configure dio_dir[%d]", i);
            PRINT_ERR_MSG(err_msg);
            ret_flag = FAIL;
        }
    } // End for i

    // Set all configured dio lines to DIO_LO
    fflush_printf("setting dio lines to DIO_LO\n");
    for (i=0; i<config.num_motor; i++) {
        // Set clk dio line to zero
        rval = comedi_dio_write(
                comedi_info[dio_dev].device,
                config.dio_subdev,
                config.dio_clk[i],
                DIO_LO
                );
        if (rval != 1) {
            snprintf(err_msg, ERR_SZ, "unable to set dio_clk[%d] to zero", i);
            PRINT_ERR_MSG(err_msg);
            ret_flag = FAIL;
        }
        // Set dir dio line to zero
        rval = comedi_dio_write(
                comedi_info[dio_dev].device,
                config.dio_subdev,
                config.dio_dir[i],
                DIO_LO
                );
        if (rval != 1) {
            snprintf(err_msg, ERR_SZ, "unable to set dio_dir[%d] to zero", i);
            PRINT_ERR_MSG(err_msg);
            ret_flag = FAIL;
        } 
    } // End for i

    // Get max data and krange for conversion to physical units
    comedi_info[ain_dev].maxdata = comedi_get_maxdata(
            comedi_info[ain_dev].device, 
            config.ain_subdev, 
            dummy_ain_chan
            );
    rval = comedi_get_krange(
            comedi_info[ain_dev].device, 
            config.ain_subdev, 
            dummy_ain_chan, 
            AIN_RANGE, 
            &(comedi_info[ain_dev].krange));
    if (rval < 0) {
        PRINT_ERR_MSG("unable to get krange");
        return FAIL;
    }
    return ret_flag;
}


// ------------------------------------------------------------------
// Function: rt_cleanup
//
// Purpose: Cleanup function for real-time task. Cleans up based on 
// cleanup level. 
// 
// level 1 => closes comedi device
// level 2 => deletes rt_task and then closes comedi device.
//
// Arguments:
//   level        = integer representing clean up level
//   comedi_info  = daq/dio device information structure
//   rt_task      = real-time task
//
// Return: SUCCESS or FAIL
//
// ------------------------------------------------------------------

int rt_cleanup(int level, comedi_info_t comedi_info[], RT_TASK *rt_task)
{
    int i;
    int ret_flag = SUCCESS;

    fflush_printf("starting rt_cleanup: level=%d\n", level);

    switch (level) {

        case RT_CLEANUP_LEVEL_2:
            fflush_printf("  %d: deleting rt_task\n", RT_CLEANUP_LEVEL_2);
            if (rt_task_delete(rt_task) != 0) {
                PRINT_ERR_MSG("unable to delete rt_task");
                ret_flag = FAIL;
            }

        case RT_CLEANUP_LEVEL_1:

            for (i=0; i<NUM_COMEDI_DEV; i++) {
                fflush_printf("  %d: closing comedi device %d\n", RT_CLEANUP_LEVEL_1, i);
                if (comedi_close(comedi_info[i].device)!=0) {
                    PRINT_ERR_MSG("unable to close comedi device");
                    ret_flag = FAIL;
                }
            }

        default:
            break;
    }

    fflush_printf("rt_cleanup complete\n");

    return ret_flag;
}

// -------------------------------------------------------------------
// Function: ain_to_phys
//
// Purpose: Converts analog input reading to physical units (volts). 
// This function is necessary because the lxrt-comedi library does not
// provide it, unlike the comedi library.
//
// Arguments:
//   data         = raw integer reading from daq card
//   comedi_info  = daq/dio device information structure
//   volts        = pointer to float for voltage value
//
// Return: SUCCESS or FAIL
//
// -------------------------------------------------------------------

int ain_to_phys(lsampl_t data, comedi_info_t comedi_info, float *volts)
{
  float max_rng;
  float min_rng;
  
  // Check if maxdata is zero
  if (comedi_info.maxdata==0) {
    PRINT_ERR_MSG("maxdata == 0"); 
    return FAIL;
  }

  // Get max and min ranges, Is this correct ??
  max_rng = ((float) (comedi_info.krange.max))*1.0e-6;
  min_rng = ((float) (comedi_info.krange.min))*1.0e-6;
  
  // Convert integer data to float voltages
  *volts = (float) data;
  *volts /= (float) comedi_info.maxdata;
  *volts *= (max_rng - min_rng);
  *volts += min_rng;

  return SUCCESS;
}

// -----------------------------------------------------------------
// Function: reassign_sigint 
//
// Purpose: Reassigns sigint signal handler and prints an error 
// message on failure. Returns either old signal handler or SIG_ERR.
//
// ------------------------------------------------------------------
sighandler_t reassign_sigint(sighandler_t sigint_func)
{
  sighandler_t sigint_func_old;
  sigint_func_old = signal(SIGINT,sigint_func);
  if (sigint_func_old == SIG_ERR) {
    PRINT_ERR_MSG("assigning SIGINT handler");
  }
  return sigint_func_old;
}

// ------------------------------------------------------------------
// Function: sigint_func
//
// Purpuse: SIGNIT signal handler. Interrupts realtime task. 
//
// ------------------------------------------------------------------
void sigint_func(int sig) {
  end = 1;
  return;
}



