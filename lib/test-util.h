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
  test-util.h

  Purpose: Definitions for unit testing
 
  Author: Will Dickson
---------------------------------------------------------------------- */
#ifndef INC_TEST_H_ 
#define INC_TEST_H_

// Configuration test parameters
#define DEV0_NAME "/dev/comedi0"
#define DEV1_NAME "/dev/comedi1"
#define AIN_DEV 1
#define AIN_SUBDEV 0
#define AIN_ZERO_NUM 100
#define AIN_ZERO_DT 0.01
#define AIN_FILT_LPCUT 2.0
#define CAL_FILE_PATH "FT8652.cal"
#define DIO_DEV 0
#define DIO_SUBDEV 2
#define DIO_CLK {0,2,4,6,8,10,12,14}
#define DIO_DIR {1,3,5,7,9,11,13,15}
#define KINE_MAP {0,1,2,3,4,5,6,7}
#define KINE_LABEL {"rotation_0", "deviation_0", "deviation_1", "rotation_1", "stroke_0", "stroke_1", "pitch", "translation"}
#define NUM_MOTOR 8
#define FF_FT {0,5}
#define FF_BASIC_TOOLTRANS {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
#define FF_DYNAM_TOOLTRANS {0,2,-1}
#define FF_MOTOR {6,7}
#define FF_MASS {1.0, 2.0}
#define FF_IND2UNIT {1.5, 2.5}
#define FF_AXESUNITS {"rad", "m"}
#define FF_DAMPING {0.0, 0.1}
#define FF_FLAG {FF_ON, FF_ON}
#define FF_INTEG_TYPE INTEG_RKUTTA
#define DT_NS 300000
#define STARTUP_T 0.0

//// Kinematics test parameters
#define KINE_NROW 50000
#define KINE_NCOL NUM_MOTOR

// Filter test parameters
#define NUM_CYCLE  50
#define TEST_CYCLE_MIN  20
#define TEST_CYCLE_MAX 40
#define F_SINE  5.0
#define NUM_TESTVALS 50
#define F_TEST_MAX (F_SINE*20.0)
#define F_TEST_MIN (F_SINE/20.0)
#define FILT_DT  (1.0/(F_SINE*1000.0))
#define FILT_GAIN_TOL 0.01

// Integrator test parameters
#define MASS 1.0
#define DAMPING 0.0
#define FORCE (-9.81*MASS)
#define INTEG_DT 0.01
#define INTEG_T 1.0
#define POS_INIT 0.0
#define VEL_INIT 50.0
#define INTEG_TOL 1.0e-6




// Initialize system configuration for testing
extern int init_test_config(config_t *config);

// Free test configuration memory
extern void free_test_config(config_t *config);

// Initialize kinematics array for testing
extern int init_test_kine(array_t *kine, config_t config);

// Free kine array memory 
extern void free_test_kine(array_t *kine);

// Initialixe data array structure for testing 
extern int init_test_data(data_t *data, int N);

// Free data array structure memory memory 
extern void free_test_data(data_t *data);

#endif // INC_TEST_H_ 
