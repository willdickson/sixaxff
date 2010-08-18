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
  devel-sixaxff.c

  Purpose: Temporary - used for development and testing.
 
  Author: Will Dickson
---------------------------------------------------------------------- */
#include <stdio.h>
#include <stdlib.h>
#include "sixaxff.h"
#include "util.h"
#include "test-util.h"

#include "sixax.h"
#include "ftconfig.h"

int main(int argc, char *argv[])
{
  array_t kine;
  config_t config;
  data_t data;
  int end_pos[MAX_MOTOR];
  int ret_val;

  // Test six axis sensor calibration stuff
  int i;
  Calibration *cal = NULL;
  char cal_file_path[] = "FT8652.cal";
  float tool_trans[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  float sample[6] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  float ft[6]; 

  sixax_init_cal(&cal,cal_file_path,tool_trans);
  sixax_print_calinfo(cal);

  sixax_sample2ft(cal,sample,ft);
  printf("ft\n");
  for (i=0; i<6; i++) {
      printf("%13.5e ", ft[i]);
  }
  printf("\n");


  float bias[6]={1.0,2.0,3.0,4.0,5.0,6.0};
  sixax_set_bias(cal,bias);
  sixax_print_calinfo(cal);
  sixax_free_cal(cal);
  // 


  // Initialize 
  init_test_config(&config);
  if (init_test_kine(&kine,config) != SUCCESS) {
    PRINT_ERR_MSG("unable to initialize kinematics");
  }
  if (init_test_data(&data,kine.nrow) != SUCCESS) {
    PRINT_ERR_MSG("unable to initialize data");
  }

  // Run sixaxff
  ret_val = sixaxff(kine,config,data,end_pos);
  
  // Clean up
  free_test_kine(&kine);
  free_test_data(&data);

 
  return 0;
}





