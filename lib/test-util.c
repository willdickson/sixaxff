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
  test-util.c

  Purpose: Contains functions for setting up unit tests for sixaxff.h
 
  Author: Will Dickson
---------------------------------------------------------------------- */
#include "sixaxff.h"
#include "util.h"
#include "test-util.h"

// -----------------------------------------------------------------
// Function: init_config
//
// Purpose: Initialize system configuration
//
// -----------------------------------------------------------------
int init_test_config(config_t *config)
{
    int i;
    char *cal_file_path = CAL_FILE_PATH;
    unsigned int dio_clk[] = DIO_CLK;
    unsigned int dio_dir[] = DIO_DIR;
    unsigned int kine_map[] = KINE_MAP;
    char *kine_label[] = KINE_LABEL;
    unsigned int ff_ft[] = FF_FT;
    unsigned int ff_motor[] = FF_MOTOR;
    float ff_tooltrans[] = FF_TOOLTRANS;
    float ff_mass[] = FF_MASS;
    float ff_ind2unit[] = FF_IND2UNIT;
    char *ff_axesunits[NUM_FF] = FF_AXESUNITS;
    float ff_damping[] = FF_DAMPING;
    unsigned int ff_flag[] = FF_FLAG;
    int flag = SUCCESS;

    config -> num_motor = NUM_MOTOR;
    (config -> dev_name)[0] = DEV0_NAME;
    (config -> dev_name)[1] = DEV1_NAME;
    config -> ain_dev = AIN_DEV;
    config -> ain_subdev = AIN_SUBDEV;
    config -> ain_zero_num = AIN_ZERO_NUM;
    config -> ain_zero_dt = AIN_ZERO_DT; 
    config -> ain_filt_lpcut = AIN_FILT_LPCUT;
    config -> cal_file_path = cal_file_path;
    config -> dio_dev = DIO_DEV;
    config -> dio_subdev = DIO_SUBDEV;
    for (i=0;i<config->num_motor;i++){
        (config->dio_clk)[i] = dio_clk[i];
        (config->dio_dir)[i] = dio_dir[i];
        (config->kine_map)[i] = kine_map[i]; 
        (config->kine_label)[i] = kine_label[i];
    }
    for (i=0; i<NUM_FF; i++) {
        (config -> ff_ft)[i] = ff_ft[i];
        (config -> ff_motor)[i] = ff_motor[i];
        (config -> ff_mass)[i] = ff_mass[i];
        (config -> ff_ind2unit)[i] = ff_ind2unit[i];
        (config -> ff_axesunits)[i] = ff_axesunits[i];
        (config -> ff_damping)[i] = ff_damping[i];
        (config -> ff_flag)[i] = ff_flag[i];
    }
    for (i=0;i<6;i++) {
        (config -> ff_tooltrans)[i] = ff_tooltrans[i];
    }
    config -> ff_integ_type = FF_INTEG_TYPE;
    config -> dt = DT_NS;
    config -> startup_t = STARTUP_T;
    return flag;
}

// ---------------------------------------------------------------
// Function: free_test_config
//
// Purpose: frees memory allocated for test configuration
//
// ----------------------------------------------------------------
void free_test_config(config_t *config)
{
    return;
}

// ---------------------------------------------------------------
// Function: init_kine
//
// Purpose: Initialize kinematics
//
// ---------------------------------------------------------------
int init_test_kine(array_t *kine, config_t config)
{
    int i;
    int j;
    int pos;
    int flag = SUCCESS;

    if (init_array(kine,KINE_NROW, KINE_NCOL, INT_ARRAY) != SUCCESS) {
        flag = FAIL;
    }

    // Create kinematics
    for (i=0; i<kine->nrow; i++) {
        for (j=0; j<kine->ncol; j++) {
            pos = i;
            if(set_array_val(*kine,i,j,&pos)==FAIL) { 
                PRINT_ERR_MSG("error writing kinematic position");
                flag = FAIL;
            }
        }
    }
    return flag;
}

// --------------------------------------------------------------
// Function: free_kine
//
// Purpose: frees memory allocated for kinematics
//
// ---------------------------------------------------------------
void free_test_kine(array_t *kine)
{
    free_array(kine);
}


// ---------------------------------------------------------------
// Function: init_data
//
// Purpose: initize return data structure
//
// ---------------------------------------------------------------
int init_test_data(data_t *data, int N)
{
    int flag = SUCCESS;

    if (init_array(&(data -> t), N, 1, DBL_ARRAY) != SUCCESS) {
        flag = FAIL;
    }
    if (init_array(&(data -> pos), N, 2, FLT_ARRAY) != SUCCESS) {
        flag = FAIL;
    }
    if (init_array(&(data -> vel), N, 2, FLT_ARRAY) != SUCCESS) {
        flag = FAIL;
    }
    if (init_array(&(data -> ft), N, 6, FLT_ARRAY) != SUCCESS) {
        flag = FAIL;
    }
    return flag;
}

// ---------------------------------------------------------------
// Function: free_data
//
// Prupose: fress memory allocated for return data
//
// ---------------------------------------------------------------
void free_test_data(data_t *data)
{
    free_array(&(data -> t));
    free_array(&(data -> pos));
    free_array(&(data -> vel));
    free_array(&(data -> ft));
}
