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
  check.h

  Purpose: 
 
  Author: Will Dickson
---------------------------------------------------------------------- */
#ifndef INC_CHECK_H_
#define INC_CHECK_H_
#include "sixaxff.h"

// Check input arguments for sixaxff function
extern int check_sixaxff_input(
    array_t kine, 
    config_t config, 
    data_t data
    );

// Check system configuration structure
extern int check_config(config_t config);

// Check ranges in system configuration
extern int check_ranges(config_t config);

// Check clk/dir configuration
extern int check_clkdir(config_t config);

// Check mapping from kinematics to motors
extern int check_kine_map(config_t config);

// Check kinematics array
extern int check_kine(array_t kine);

// Check compatibility between configuration and kinematics
extern int check_kine_compat(config_t config, array_t kine);

// Check array structure
extern int check_array(array_t array);

// Check data structure
extern int check_data(data_t data);

// Check that kinematics and data structures are compatible
extern int check_data_compat(array_t kine, data_t data);

#endif // INC_CHECK_H_
