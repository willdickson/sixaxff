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
  test-sixaxff.c

  Purpose: Set of unit tests for the sixaxff library

  Author: Will Dickson 
---------------------------------------------------------------------- */
#include <stdio.h>
#include <stdlib.h>
#include <CUnit/Basic.h>
#include "sixaxff.h"
#include "util.h"
#include "check.h"
#include "test-util.h"

// Function prototypes ------------------------------------------------
int testsuite_setup(void);
int testsuite_teardown(void);

void test_init_array(void);
void test_set_array_val(void);
void test_get_array_val(void);

void test_check_config(void);
void test_check_kine(void);
void test_check_kine_compat(void);
void test_check_data(void);
void test_check_data_compat(void);
void test_check_sixaxff_input(void);

void test_lowpass_filt1(void);
void test_integrator(void);


//----- --------------------------------------------------------------
// Function: main
//
// Purpose: sets up unit testing framework and runs the tests.
//
// -------------------------------------------------------------------
int main(int argc, char *argv[]) 
{
  CU_TestInfo array_test_array[] = {
    {"init_array",    test_init_array},
    {"set_array_val", test_set_array_val},
    {"get_array_val", test_get_array_val},
    CU_TEST_INFO_NULL,
  };
  CU_TestInfo check_test_array[] = {
    {"check_config",      test_check_config},
    {"check_kine",        test_check_kine},
    {"check_kine_compat", test_check_kine_compat},
    {"check_data",        test_check_data},
    {"check_data_compat", test_check_data_compat},
    {"check_sixaxff_input", test_check_sixaxff_input},
    CU_TEST_INFO_NULL,
  };
  CU_TestInfo sixaxff_test_array[] = {
    {"lowpass_filt1", test_lowpass_filt1},
    {"integrator",    test_integrator},
    CU_TEST_INFO_NULL,
  }; 
  CU_SuiteInfo suites[] = {
    {"array testsuite", testsuite_setup, testsuite_teardown, array_test_array},
    {"check testsuite", testsuite_setup, testsuite_teardown, check_test_array},
    {"sixaxff testsuite", testsuite_setup, testsuite_teardown, sixaxff_test_array},
    CU_SUITE_INFO_NULL,
  };
  
  // Initialize the CUnit test registry and register test suites 
  if (CU_initialize_registry() != CUE_SUCCESS) {
    fprintf(stderr, "Cunit Error: %s\n", CU_get_error_msg());
    return CU_get_error();
  }
  if (CU_register_suites(suites) != CUE_SUCCESS) {
    fprintf(stderr, "Cunit Error: %s\n", CU_get_error_msg());
    return CU_get_error();
  }
  
  // Run unit tests
  CU_basic_set_mode(CU_BRM_VERBOSE);
  freopen("/dev/null", "w", stderr); // Redirect stderr to get rid of annoying message
  CU_basic_run_tests();
  freopen("/dev/tty", "w", stderr);  // Return stderr to terminal

  // Clean up
  CU_cleanup_registry();
  return CU_get_error();

}

// ------------------------------------------------------------------
// Function: testsuit_setup
//
// Puropse: dummy testsuite setup function
//
// ------------------------------------------------------------------
int testsuite_setup(void) {
  return 0;
}

// ------------------------------------------------------------------
// Function: testsuite_teardown
// 
// Purpose: dummy testsuite teardown function
//
// ------------------------------------------------------------------
int testsuite_teardown(void) {
  return 0;
}


// ------------------------------------------------------------------
// Function: test_init_array
//
// Purpose: Unit test for init_array function
//
// ------------------------------------------------------------------
void test_init_array(void)
{
  int nrow = 20;
  int ncol = 50;
  array_t array;
  
  // Int array creation
  CU_ASSERT(init_array(&array,nrow,ncol,INT_ARRAY)==SUCCESS);
  free_array(&array);

  // Float array creation
  CU_ASSERT(init_array(&array,nrow,ncol,FLT_ARRAY)==SUCCESS);
  free_array(&array);

  // Unkown array type creation - should fail
  CU_ASSERT(init_array(&array,nrow,ncol,UNKNOWN_ARRAY)==FAIL);
  
  // Bad nrow - should fail
  CU_ASSERT(init_array(&array,-1,ncol,UNKNOWN_ARRAY)==FAIL);
  
  // Bad ncol - should fail
  CU_ASSERT(init_array(&array,nrow,-1,UNKNOWN_ARRAY)==FAIL);
  
  return;
}

// ------------------------------------------------------------------
// Function: test_set_array_val
//
// Purpose: Unit test for set_array_val function
//
// ------------------------------------------------------------------
void test_set_array_val(void)
{
  int i,j;
  int s0,s1;
  int nrow = 15;
  int ncol = 10;
  array_t array;
  int ival;
  float fval;

  // Test integer arrays
  CU_ASSERT(init_array(&array,nrow,ncol,INT_ARRAY)==SUCCESS);
  for (i=0; i<nrow; i++) {
    for (j=0; j<ncol; j++) {
      ival = i + j;
      CU_ASSERT(set_array_val(array,i,j,&ival)==SUCCESS);
    }
  }

  s0 = array.s0;
  s1 = array.s1;
  for (i=0;i<nrow;i++) {
    for (j=0;j<ncol;j++) {
      ival = *((int*)( (array.data) + i*s0 + j*s1));
      CU_ASSERT(ival==i+j);
    } 
  }
  
  // Set values out of bounds - should FAIL 
  CU_ASSERT(set_array_val(array,nrow,0,0)==FAIL);
  CU_ASSERT(set_array_val(array,-1,0,0)==FAIL);
  CU_ASSERT(set_array_val(array,0,ncol,0)==FAIL);
  CU_ASSERT(set_array_val(array,0,-1,0)==FAIL);

  free_array(&array);

  // Test Float arrays
  CU_ASSERT(init_array(&array,nrow,ncol,FLT_ARRAY)==SUCCESS);
  for (i=0; i<nrow; i++) {
    for (j=0; j<ncol; j++) {
      fval = 10.0*(i + j);
      CU_ASSERT(set_array_val(array,i,j,&fval)==SUCCESS);
    }
  }

  s0 = array.s0;
  s1 = array.s1;
  for (i=0;i<nrow;i++) {
    for (j=0;j<ncol;j++) {
      fval = *((float*)( (array.data) + i*s0 + j*s1));
      CU_ASSERT(fval==10.0*(i+j));
    } 
  }
  free_array(&array);

  return;
}


// ------------------------------------------------------------------
// Function: test_get_array_val
//
// Purpose: Unit test for get_array_val function
//
// ------------------------------------------------------------------
void test_get_array_val(void)
{
  int i,j;
  int s0,s1;
  int nrow = 17;
  int ncol = 11;
  array_t array;
  int ival;
  float fval;

  // Test integer arrays
  init_array(&array,nrow,ncol,INT_ARRAY);
  for (i=0; i<nrow;i++) {
    for (j=0; j<ncol;j++) {
      ival = 2*i*j;
      CU_ASSERT(set_array_val(array,i,j,&ival)==SUCCESS);
    }
  }
  
  s0 = array.s0;
  s1 = array.s1;
  for (i=0;i<nrow;i++) {
    for (j=0;j<ncol; j++) {
      CU_ASSERT(get_array_val(array,i,j,&ival)==SUCCESS);
      CU_ASSERT(ival == 2*i*j);
    }
  }

  // Get values out of bounds - should fail
  CU_ASSERT(get_array_val(array,nrow,0,0)==FAIL);
  CU_ASSERT(get_array_val(array,-1,0,0)==FAIL);
  CU_ASSERT(get_array_val(array,0,ncol,0)==FAIL);
  CU_ASSERT(get_array_val(array,0,-1,0)==FAIL);
  
  free_array(&array);

  // Test float arrays
  init_array(&array,nrow,ncol,FLT_ARRAY);
  for (i=0; i<nrow;i++) {
    for (j=0; j<ncol;j++) {
      fval = 0.5*i*j;
      CU_ASSERT(set_array_val(array,i,j,&fval)==SUCCESS);
    }
  }
  
  s0 = array.s0;
  s1 = array.s1;
  for (i=0;i<nrow;i++) {
    for (j=0;j<ncol; j++) {
      CU_ASSERT(get_array_val(array,i,j,&fval)==SUCCESS);
      CU_ASSERT(fval == 0.5*i*j);
    }
  }
  free_array(&array);
}


// ------------------------------------------------------------------
// Function: test_check_config 
//
// Purpose: Unit test for the check_config function.
//
// ------------------------------------------------------------------
void test_check_config(void)
{
  config_t config;
  config_t config_test;

  CU_ASSERT(init_test_config(&config)==SUCCESS);

  // Test for pass on good config
  CU_ASSERT(check_config(config)==SUCCESS);

  // Test for Fail on bad CLK/DIR 
  config_test = config;
  config_test.dio_clk[0] = config_test.dio_dir[1];
  CU_ASSERT_FALSE(check_config(config_test)==SUCCESS);

  // Test for fail on bad kine map
  config_test = config;
  config_test.kine_map[0] = config_test.kine_map[2];
  CU_ASSERT_FALSE(check_config(config_test)==SUCCESS);

  // DIO disable range test
  config_test = config;
  config_test.dio_disable = 25;
  CU_ASSERT_FALSE(check_config(config_test)==SUCCESS);

  // DIO disable clk/dir collision test
  config_test = config;
  config_test.dio_disable = config_test.dio_clk[1];
  CU_ASSERT_FALSE(check_config(config_test)==SUCCESS);
  config_test = config;
  config_test.dio_disable = config_test.dio_dir[0];
  CU_ASSERT_FALSE(check_config(config_test)==SUCCESS);

  // Analog input range test
  config_test = config;
  config_test.yaw_ain = MAX_AIN+1;
  CU_ASSERT_FALSE(check_config(config_test)==SUCCESS);
  
  // Analog input zeroing sample interval test
  config_test = config;
  config_test.yaw_ain_zero_dt = 0.5*AIN_ZERO_DT_MIN;
  CU_ASSERT_FALSE(check_config(config_test)==SUCCESS);

  // Analog input zeroing number of samples test
  config_test = config;
  config_test.yaw_ain_zero_num = 0;
  CU_ASSERT_FALSE(check_config(config_test)==SUCCESS);
  
  // Inertia range test
  config_test = config;
  config_test.yaw_inertia = 0;
  CU_ASSERT_FALSE(check_config(config_test)==SUCCESS);

  // Yaw index to degree conversion test
  config_test = config;
  config_test.yaw_ind2deg = 0.5*FLT_EPSILON;
  CU_ASSERT_FALSE(check_config(config_test)==SUCCESS);

  // Torque limit range test
  config_test = config;
  config_test.yaw_torq_lim = MIN_TORQ_LIM - 0.1;
  CU_ASSERT_FALSE(check_config(config_test)==SUCCESS);

  // Torque deadband range test
  config_test = config;
  config_test.yaw_torq_deadband = -1.0;
  CU_ASSERT_FALSE(check_config(config_test)==SUCCESS);

  // Yaw filter lowpass cutoff test
  config_test = config;
  config_test.yaw_filt_lpcut = -1.0;
  CU_ASSERT_FALSE(check_config(config_test)==SUCCESS);

  // Yaw filter highpass cutoff test
  config_test = config;
  config_test.yaw_filt_hpcut = -1.0;
  CU_ASSERT_FALSE(check_config(config_test)==SUCCESS);

  // Yaw damping constant
  config_test = config;
  config_test.yaw_damping = -1.0;
  CU_ASSERT_FALSE(check_config(config_test)==SUCCESS);
  
  // dt range tests
  config_test = config;
  config_test.dt = MAX_DT_NS + 1;
  CU_ASSERT_FALSE(check_config(config_test)==SUCCESS);
  config_test.dt = MIN_DT_NS - 1;
  CU_ASSERT_FALSE(check_config(config_test)==SUCCESS);  

  // Integrator type test
  config_test = config;
  config_test.dt = INTEG_UNKNOWN;
  CU_ASSERT_FALSE(check_config(config_test)==SUCCESS);   

  // Free memory allocated for config
  free_test_config(&config); 

}

// -------------------------------------------------------------
// Function: test_check_kine
//
// Purpose: unit test for check_kine function
//
// -------------------------------------------------------------
void test_check_kine(void)
{
  int i,j;
  int nrow = 100;
  int ncol = 5;
  array_t  kine;
  int val;

  CU_ASSERT(init_array(&kine,nrow,ncol,INT_ARRAY)==SUCCESS);
  
  // Kinematics which should pass
  for (i=0; i<kine.nrow; i++) {
    for (j=0; j<kine.ncol; j++) {
      val = i + j;
      CU_ASSERT_FATAL(set_array_val(kine,i,j,&val)==SUCCESS);
    }
  }
  CU_ASSERT(check_kine(kine)==SUCCESS);

  // Kinematics which should fail because too big step
  for (i=0; i<kine.nrow; i++) {
    for (j=0; j<kine.ncol; j++) {
      val = i*(j+1);
      CU_ASSERT_FATAL(set_array_val(kine,i,j,&val)==SUCCESS);
    }
  }
  CU_ASSERT_FALSE(check_kine(kine)==SUCCESS);
  
  // Set to ok kinematics
  for (i=0; i<kine.nrow; i++) {
    for (j=0; j<kine.ncol; j++) {
      val = i + j;
      set_array_val(kine,i,j,&val);
    }
  }
  // Kinematics which should fail with bad # of rows
  kine.nrow = 0;
  CU_ASSERT_FALSE(check_kine(kine)==SUCCESS);
  // Kinematics which should fail with bad # columns
  kine.nrow = nrow;
  kine.ncol = -3;
  CU_ASSERT_FALSE(check_kine(kine)==SUCCESS);
  // Kinematics which should fail with bad stride s0
  kine.ncol = ncol;
  kine.s0 = 0;
  CU_ASSERT_FALSE(check_kine(kine)==SUCCESS);
  // Kinematics which should fail with bad stride s1
  kine.s0 = ncol*sizeof(int);
  kine.s1 = 0;
  CU_ASSERT_FALSE(check_kine(kine)==SUCCESS);
  // Should fail with unkown type
  kine.s1 = sizeof(int);
  kine.type = UNKNOWN_ARRAY;
  CU_ASSERT_FALSE(check_kine(kine)==SUCCESS);

  // Clean up
  free_array(&kine);
  return;
}


// ----------------------------------------------------------------
// Function: test_check_kine_compat
//
// Purpose: unit test for check_compat function
//
// -----------------------------------------------------------------
void test_check_kine_compat(void)
{
  array_t kine;
  config_t config;

  // Setup config structure
  init_test_config(&config);

  // Setup kinematics
  init_test_kine(&kine, config);
    
  // Kinematics and config which should be compatible
  CU_ASSERT(check_kine_compat(config,kine)==SUCCESS);

  // Kinematics and config which shouldn't be compatible
  config.num_motor = KINE_NCOL+2;
  CU_ASSERT_FALSE(check_kine_compat(config,kine)==SUCCESS);
  config.num_motor = KINE_NCOL;

  free_array(&kine);
  free_test_config(&config);

  return;
}

// ------------------------------------------------------------------
// Function: test_check_data
//
// Purpose: unit test of check_data function
// ------------------------------------------------------------------
void test_check_data(void)
{
  int N = 100;
  data_t data;
  data_t test_data;

  CU_ASSERT(init_test_data(&data,N)==SUCCESS);

  CU_ASSERT(check_data(data)==SUCCESS);

  // Test column check
  test_data = data;
  test_data.t.ncol = 2;
  CU_ASSERT(check_data(test_data) == FAIL);
  test_data = data;
  test_data.pos.ncol = 2;
  CU_ASSERT(check_data(test_data) == FAIL);
  test_data = data;
  test_data.vel.ncol = 2;
  CU_ASSERT(check_data(test_data) == FAIL);
  test_data = data;
  test_data.torq.ncol = 3;
  CU_ASSERT(check_data(test_data) == FAIL);

  // Test type check
  test_data = data;
  test_data.t.type = INT_ARRAY;
  CU_ASSERT(check_data(test_data) == FAIL);
  test_data = data;
  test_data.pos.type = INT_ARRAY;
  CU_ASSERT(check_data(test_data) == FAIL);
  test_data = data;
  test_data.vel.type = INT_ARRAY;
  CU_ASSERT(check_data(test_data) == FAIL);
  test_data = data;
  test_data.torq.type = INT_ARRAY;
  CU_ASSERT(check_data(test_data) == FAIL);
  
  // Clean up
  free_test_data(&data);
  
  return;
}

// ------------------------------------------------------------------
// Function: test_check_data_compat
//
// Purpose: Unit test for check_data_compat function
//
// ------------------------------------------------------------------
void test_check_data_compat(void)
{
  config_t config;
  array_t kine;
  data_t data;
  data_t test_data;
  
  // Set up kine and data structures
  init_test_config(&config);
  CU_ASSERT(init_test_kine(&kine,config)==SUCCESS);
  CU_ASSERT(init_test_data(&data,kine.nrow)==SUCCESS);

  // Check for compatibility 
  CU_ASSERT(check_data_compat(kine,data)==SUCCESS);
  
  // Check for incompatbility
  test_data = data;
  test_data.t.nrow = 2;
  CU_ASSERT(check_data_compat(kine,test_data)==FAIL);
  test_data = data;
  test_data.pos.nrow = -1;
  CU_ASSERT(check_data_compat(kine,test_data)==FAIL);
  test_data = data;
  test_data.vel.nrow = 20;
  CU_ASSERT(check_data_compat(kine,test_data)==FAIL);
  test_data = data;
  test_data.torq.nrow = 8;
  CU_ASSERT(check_data_compat(kine,test_data)==FAIL);
  
  // Clean up
  free_test_data(&data);
  free_test_kine(&kine);
  free_test_config(&config);

  return;
}

// ------------------------------------------------------------------
// Fucntion: test_check_sixaxff_input
//
// Purpose: Unit test of check_sixaxff_input function
//
// ------------------------------------------------------------------
void test_check_sixaxff_input(void)
{
  config_t config, test_config;
  array_t kine, test_kine;
  data_t data, test_data;
  
  // Intialize test inputs
  init_test_config(&config);
  CU_ASSERT(init_test_kine(&kine,config) == SUCCESS);
  CU_ASSERT(init_test_data(&data,kine.nrow) == SUCCESS); 

  // Try with passing data
  CU_ASSERT(check_sixaxff_input(kine,config,data)==SUCCESS);

  // Try w/ bad configuration
  test_config = config;
  test_config.num_motor = -1;
  CU_ASSERT(check_sixaxff_input(kine,test_config,data)==FAIL);
  
  // Try w/ bad kinematics
  test_kine = kine;
  test_kine.ncol = -1;
  CU_ASSERT(check_sixaxff_input(test_kine,config,data)==FAIL);

  // Try w/ bad data
  test_data = data;
  test_data.t.ncol = -1;
  CU_ASSERT(check_sixaxff_input(kine,config,test_data)==FAIL);
  
  // Try with incompatible kine and config
  test_kine = kine;
  test_config = config;
  test_kine.ncol = test_config.num_motor+1;
  CU_ASSERT(check_sixaxff_input(test_kine,test_config,data)==FAIL);
  
  // Try w/ incompatible kine and data
  test_data = data;
  test_kine = kine;
  test_data.pos.nrow = test_kine.nrow+1;
  CU_ASSERT(check_sixaxff_input(test_kine,config,test_data)==FAIL);

  // Clean up
  free_test_data(&data);
  free_test_kine(&kine);
  free_test_config(&config);

  return;
}

// ------------------------------------------------------------------
// Function: test_lowpass_filt1
//
// Purpose: Unit test for lowpass_filt1 function. Currently only 
// tests the gain of the filter against the theoretical value. 
//
// -------------------------------------------------------------------
void test_lowpass_filt1(void)
{
  float f_cut;
  float t;
  float x;
  float y;
  float x_max;
  float y_max ;
  float gain_meas;
  float gain_calc;
  float gain_err ;
  float a,b;
  int i,j;

  // Loop over test values
  for (j=0; j<NUM_TESTVALS; j++) {

    // Check sinewave with freq << freq_cut
    t = 0;
    y = 0;
    a = (F_TEST_MAX - F_TEST_MIN)/((float) NUM_TESTVALS);
    b = F_TEST_MIN;
    f_cut = a*((float) j) + b;
    x_max = 0;
    y_max = 0;


    for (i=0; i< (int)(NUM_CYCLE/(F_SINE*FILT_DT)); i++) {
      x = sinf(2*M_PI*t*F_SINE);
      y = lowpass_filt1(x,y,f_cut,FILT_DT);
      t+=FILT_DT;
      // Get max values of x and y for amplitude estimate
      if ((i>(int)(TEST_CYCLE_MIN/(F_SINE*FILT_DT))) && 
	  (i< (int)(TEST_CYCLE_MAX/(F_SINE*FILT_DT)))) {
	x_max = x > x_max ? x : x_max;
	y_max = y > y_max ? y : y_max;
      }
    }
    
    // Compute the gain error
    gain_meas = y_max/x_max;
    gain_calc = 1.0/sqrtf(1.0 + pow(F_SINE/f_cut,2));
    gain_err = fabs(gain_meas - gain_calc)/gain_calc;    
    CU_ASSERT(gain_err < FILT_GAIN_TOL);
  };

  return;
}

// -----------------------------------------------------------------
// Function: test_integrator
//
// Purpose: Unit test for integrator function. Note, this is
// test is kind of cheesey as it simple integrates a simple
// ballistic trajectory and 
//
// -----------------------------------------------------------------
void test_integrator(void)
{
  int N = (int)(INTEG_T/INTEG_DT);
  state_t state_kutta[N];
  state_t state_euler[N];
  float pos;
  float vel;
  float err_kutta;
  float err_euler;
  float pos_max;
  float pos_min;
  float t ;
  int i;
  int ret_val;
  int test_flag;

  // Initial conditions
  state_kutta[0].pos = POS_INIT;
  state_kutta[0].vel = VEL_INIT;
  state_euler[0].pos = POS_INIT;
  state_euler[0].vel = VEL_INIT;

  // Integrate test case
  test_flag = SUCCESS;
  for (i=0; i<N-1; i++) {

    // Runge-Kutta Integration
    ret_val = integrator(
			 state_kutta[i],
			 &state_kutta[i+1],
			 FORCE,
			 MASS,
			 DAMPING,
			 INTEG_DT,
			 INTEG_RKUTTA
			 );
    if (ret_val == FAIL) {
      test_flag = FAIL;
    }

    // Euler Integration
    ret_val = integrator(
			 state_euler[i],
			 &state_euler[i+1],
			 FORCE,
			 MASS,
			 DAMPING,
			 INTEG_DT,
			 INTEG_EULER
			 );
    if (ret_val == FAIL) {
      test_flag = FAIL;
    }
  }
  CU_ASSERT(test_flag==SUCCESS);

  // Check test case against exact solution
  t = 0.0;
  err_kutta = 0.0;
  err_euler = 0.0;
  pos_max = POS_INIT;
  pos_min = POS_INIT;
  for (i=1; i<N; i++) {
    t += INTEG_DT;
    // Exact solution for pos and vel
    pos = POS_INIT + VEL_INIT*t + 0.5*(FORCE/MASS)*pow(t,2);
    vel = VEL_INIT + (FORCE/MASS)*t;
    
    // Compute absolute cummulative err 
    err_kutta += fabs(state_kutta[i].pos - pos);
    err_euler += fabs(state_euler[i].pos - pos);

    // Get max and min positions for total displacement
    pos_max = pos_max>pos? pos_max : pos;
    pos_min = pos_min<pos? pos_min : pos;    
  }
  
  // Compute mean cummulative error divided by total displacement
  err_kutta = err_kutta/(float)(N-1);
  err_euler = err_euler/(float)(N-1);
  err_kutta = err_kutta/(pos_max - pos_min);
  err_euler = err_kutta/(pos_max - pos_min);

  CU_ASSERT(err_kutta < INTEG_TOL);
  CU_ASSERT(err_euler < INTEG_TOL);

  // Check for failure when given unknown integrator number
  ret_val = integrator(
		       state_euler[0],
		       &state_euler[1],
		       FORCE,
		       MASS,
		       DAMPING,
		       INTEG_DT,
		       INTEG_UNKNOWN
		       );
  CU_ASSERT_FALSE(ret_val == SUCCESS);  
  return;
}

